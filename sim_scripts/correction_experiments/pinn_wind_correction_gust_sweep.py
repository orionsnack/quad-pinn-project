"""
pinn_wind_correction_sweep.py(고정 바람 5조건)의 gust 버전.
지금까지 PINN 가속도 피드포워드 보정의 A/B 검증은 전부 "트라이얼 내내 바람이
고정"인 조건에서만 했음 (12-5절). 실제 바람은 계속 변하므로, 바람이 사인파로
계속 출렁이는 gust 조건에서도 보정이 여전히 도움이 되는지 확인하는 스크립트.

바람 모델 (조건마다 고정, wind_gust_sweep.py의 방향은 유지하되 진폭만 사인파로 출렁임):
  speed(t) = base_speed + base_speed*AMP_FRACTION*sin(2*pi*t/PERIOD_S)
  방향은 base_vx/base_vy로 고정 (pinn_wind_correction_sweep.py의 조건과 동일 방향).
Gazebo에는 GUST_UPDATE_INTERVAL_S 간격으로만 갱신(계단식 근사, wind_gust_sweep.py와
동일한 이유 - gz topic pub 프로세스 spawn 비용).

주의: gust 조건에서는 PINN의 풍속 추정 오차 자체가 이미 크다는 게 알려져 있음
(EXPERIMENTS.md 12-6절: 고정바람 0.37~0.5m/s -> gust 포함 1.8m/s). 따라서 이 실험은
"추정이 부정확한 상황에서도 보정이 여전히 순이익인가"를 확인하는 것이지, 고정바람
수준의 개선율을 기대하는 게 아님.

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import asyncio
import csv
import datetime
import math
import sys
import time
from pathlib import Path

import torch
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw, AccelerationNed)

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "offline_training"))
from wind_pinn_model import WindPINN, WINDOW, FEATURES, yaw_decompose  # noqa: E402


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
# (label, base_vx, base_vy, amp_fraction, period_s)
# base_vx/base_vy는 pinn_wind_correction_sweep.py의 light/default/strong/crosswind와
# 동일한 방향 - 고정바람 결과와 나란히 비교하기 위함. calm은 base_speed=0이라 방향이
# 정의 안 되므로 gust 버전에서는 제외.
GUST_CONDITIONS = [
    ("light_gust", 2.0, 1.0, 0.6, 6.0),
    ("default_gust", 5.0, 2.0, 0.6, 6.0),
    ("strong_gust", 8.0, 3.0, 0.6, 6.0),
    ("crosswind_gust", 0.0, 6.0, 0.6, 6.0),
]
ACCEL_GAIN = 0.05  # 12-31절: 혼합 재학습 모델 기준 재스윕 결과 0.15는 default_fixed에서
MAX_ACCEL_MPS2 = 2.0    # 이미 손해(-24.8%)였고 0.05가 default/strong 양쪽에서 최선으로 확인됨
WIND_DEADBAND_MPS = 1.0  # deadband는 그대로 유지 - light_gust에서 여전히 최선(+26.1%, 12-31절)
TRIAL_DURATION_S = 18.0  # period 6s * 3주기
GUST_UPDATE_INTERVAL_S = 1.0
CALM_SETTLE_S = 5.0   # 12-23절: 3.0으로는 반복측정 시 분산이 극심함(최대 20배) 확인, 5.0으로 상향
PHASE_GAP_S = 4.0     # 12-23절: 2.0으로는 반복측정 시 분산이 극심함 확인, 4.0으로 상향
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

MODEL_PATH = Path(__file__).parent.parent.parent / "offline_training" / "wind_estimator.pt"


async def set_wind(vx, vy, vz=0.0):
    """gz topic pub은 1회성 CLI라, gz-transport 구독자 탐색(discovery, 보통
    수백ms 걸림)이 끝나기 전에 프로세스가 끝나면 CLI는 성공(exit 0)을 보고해도
    실제로 아무도 못 받을 수 있음 - 25회 반복측정 중 1회 실제로 발생 확인됨
    (EXPERIMENTS.md 12-23절). 같은 값(멱등)을 짧은 간격으로 3번 재전송해서
    완화 - 한 번이라도 discovery 이후에 도착하면 됨."""
    last_returncode = None
    for attempt in range(3):
        proc = await asyncio.create_subprocess_exec(
            "gz", "topic", "-t", f"/world/{WORLD_NAME}/wind",
            "-m", "gz.msgs.Wind",
            "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
        )
        await proc.wait()
        last_returncode = proc.returncode
        if attempt < 2:
            await asyncio.sleep(0.15)
    if last_returncode != 0:
        # 조용히 넘기면 바람이 실제로 안 걸린 채 트라이얼이 진행될 수 있음 - 실제로
        # 이걸로 엉터리 결과가 나온 적 있어(EXPERIMENTS.md 12-17/12-21절) 예외로 드러냄.
        raise RuntimeError(f"gz topic pub 실패 (returncode={last_returncode})")


def wind_at(cond, t):
    """cond: (label, base_vx, base_vy, amp_fraction, period_s). 방향은 base_vx/base_vy
    고정, 크기만 사인파로 진동. speed는 amp_fraction<1이면 항상 양수 유지됨."""
    _, base_vx, base_vy, amp_fraction, period_s = cond
    base_speed = math.hypot(base_vx, base_vy)
    if base_speed < 1e-9:
        return 0.0, 0.0
    ux, uy = base_vx / base_speed, base_vy / base_speed
    speed = base_speed * (1.0 + amp_fraction * math.sin(2 * math.pi * t / period_s))
    speed = max(speed, 0.0)
    return speed * ux, speed * uy


class WindCorrector:
    def __init__(self, model_path):
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        self.model = WindPINN(ckpt["window"], len(ckpt["features"]))
        self.model.load_state_dict(ckpt["model_state"])
        self.model.eval()
        self.X_mean = torch.tensor(ckpt["X_mean"], dtype=torch.float32)
        self.X_std = torch.tensor(ckpt["X_std"], dtype=torch.float32)
        self.window = ckpt["window"]
        self.features = ckpt["features"]
        self.buffer = []

    def push_state(self, feat_vec):
        self.buffer.append(feat_vec)
        if len(self.buffer) > self.window:
            self.buffer.pop(0)

    def ready(self):
        return len(self.buffer) == self.window

    @torch.no_grad()
    def estimate_wind_ned(self):
        x = torch.tensor(self.buffer, dtype=torch.float32).flatten()
        xn = (x - self.X_mean) / self.X_std
        wind_enu = self.model(xn.unsqueeze(0)).squeeze(0)
        return wind_enu[1].item(), wind_enu[0].item()  # (north, east)


class EmaSmoother:
    def __init__(self, time_constant_s, dt_s):
        self.alpha = dt_s / (time_constant_s + dt_s)
        self.value = None

    def update(self, x):
        self.value = x if self.value is None else self.alpha * x + (1 - self.alpha) * self.value
        return self.value


def apply_deadband(wind_n, wind_e, deadband):
    """speed<=deadband면 (0,0), 그 이상이면 방향 유지한 채 크기만
    (speed-deadband)만큼으로 줄임 - 문턱값에서 뚝 끊기지 않고 연속적으로 이어짐."""
    speed = (wind_n ** 2 + wind_e ** 2) ** 0.5
    if speed <= deadband or speed == 0.0:
        return 0.0, 0.0
    scale = (speed - deadband) / speed
    return wind_n * scale, wind_e * scale


async def run():
    drone = System()

    print("PX4 SITL에 연결 시도 중...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-> 드론에 연결됨!")
            break

    print("GPS/홈 위치 확인 중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-> 전역 위치 및 홈 위치 준비 완료")
            break

    print(f"\n모델 로드: {MODEL_PATH}")
    corrector = WindCorrector(MODEL_PATH)
    print(f"  window={corrector.window}  features={corrector.features}")

    await set_wind(0.0, 0.0)

    print("\n--- Arming ---")
    await drone.action.arm()
    print("-> Armed")

    print("\n--- Takeoff ---")
    await drone.action.takeoff()

    print(f"  안전 고도({SAFE_ALTITUDE_M}m) 도달 대기 중...")
    t_start = time.monotonic()
    reached_altitude = False
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        if alt >= SAFE_ALTITUDE_M:
            print(f"  -> 안전 고도 도달 (relative_altitude={alt:.2f}m)")
            reached_altitude = True
            break
        if time.monotonic() - t_start > TAKEOFF_TIMEOUT_S:
            print(f"  [경고] {TAKEOFF_TIMEOUT_S:.0f}초 내에 안전 고도 미도달.")
            break
    if not reached_altitude:
        await drone.action.land()
        return

    latest_pv = {"north": None, "east": None, "down": None, "vn": None, "ve": None, "vd": None}

    async def pv_monitor():
        async for pv in drone.telemetry.position_velocity_ned():
            latest_pv["north"] = pv.position.north_m
            latest_pv["east"] = pv.position.east_m
            latest_pv["down"] = pv.position.down_m
            latest_pv["vn"] = pv.velocity.north_m_s
            latest_pv["ve"] = pv.velocity.east_m_s
            latest_pv["vd"] = pv.velocity.down_m_s

    pv_task = asyncio.create_task(pv_monitor())
    while latest_pv["north"] is None:
        await asyncio.sleep(0.05)

    latest_att = {"roll": None, "pitch": None, "yaw": None}

    async def att_monitor():
        async for attitude in drone.telemetry.attitude_euler():
            latest_att["roll"] = attitude.roll_deg
            latest_att["pitch"] = attitude.pitch_deg
            latest_att["yaw"] = attitude.yaw_deg

    att_task = asyncio.create_task(att_monitor())
    while latest_att["roll"] is None:
        await asyncio.sleep(0.05)

    latest_gyro = {"wx": None, "wy": None, "wz": None}

    async def gyro_monitor():
        async for av in drone.telemetry.attitude_angular_velocity_body():
            latest_gyro["wx"] = av.roll_rad_s
            latest_gyro["wy"] = av.pitch_rad_s
            latest_gyro["wz"] = av.yaw_rad_s

    gyro_task = asyncio.create_task(gyro_monitor())
    while latest_gyro["wx"] is None:
        await asyncio.sleep(0.05)

    nominal = {"north": latest_pv["north"], "east": latest_pv["east"],
               "down": latest_pv["down"], "yaw": latest_att["yaw"]}
    current_cmd = {"accel_n": 0.0, "accel_e": 0.0}
    send_gaps = []

    async def offboard_sender():
        next_tick = time.monotonic()
        last_sent = None
        while True:
            try:
                await drone.offboard.set_position_velocity_acceleration_ned(
                    PositionNedYaw(nominal["north"], nominal["east"],
                                    nominal["down"], nominal["yaw"]),
                    VelocityNedYaw(0.0, 0.0, 0.0, nominal["yaw"]),
                    AccelerationNed(current_cmd["accel_n"], current_cmd["accel_e"], 0.0),
                )
            except Exception as exc:
                print(f"  [!!! offboard_sender 예외] {type(exc).__name__}: {exc}")
            now = time.monotonic()
            if last_sent is not None:
                send_gaps.append(now - last_sent)
            last_sent = now
            next_tick += SEND_PERIOD_S
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    print("\n--- Offboard 진입 준비: 초기 setpoint(현재 위치 고정) 전송 ---")
    await drone.offboard.set_position_velocity_acceleration_ned(
        PositionNedYaw(nominal["north"], nominal["east"], nominal["down"], nominal["yaw"]),
        VelocityNedYaw(0.0, 0.0, 0.0, nominal["yaw"]),
        AccelerationNed(0.0, 0.0, 0.0),
    )

    print("\n--- Offboard 모드 시작 ---")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    sender_task = asyncio.create_task(offboard_sender())

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"../../logs/pinn_correction_gust_sweep_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "wind_label", "phase", "t_s", "true_wind_north_m_s", "true_wind_east_m_s",
        "wind_est_north", "wind_est_east",
        "accel_n_m_s2", "accel_e_m_s2",
        "actual_north_m", "actual_east_m", "pos_error_m",
        "roll_deg", "pitch_deg",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    updates_per_log = max(1, round(GUST_UPDATE_INTERVAL_S / LOG_INTERVAL_S))

    async def run_trial(wind_label, phase_name, use_correction, cond):
        corrector.buffer.clear()
        smoother_n = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        smoother_e = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
        next_log = time.monotonic()
        peak_error = 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            true_vx, true_vy = wind_at(cond, t)  # ENU (x=East, y=North)
            if i % updates_per_log == 0:
                await set_wind(true_vx, true_vy)

            north, east = latest_pv["north"], latest_pv["east"]
            vn, ve = latest_pv["vn"], latest_pv["ve"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]

            r_cos, r_sin, p_cos, p_sin = yaw_decompose(roll, pitch, yaw)
            feat = {"vn_m_s": vn, "ve_m_s": ve, "roll_cos_yaw": r_cos, "roll_sin_yaw": r_sin,
                    "pitch_cos_yaw": p_cos, "pitch_sin_yaw": p_sin,
                    "wx_rad_s": latest_gyro["wx"], "wy_rad_s": latest_gyro["wy"],
                    "wz_rad_s": latest_gyro["wz"]}
            corrector.push_state([feat[f] for f in FEATURES])

            wind_n, wind_e = 0.0, 0.0
            if use_correction and corrector.ready():
                raw_n, raw_e = corrector.estimate_wind_ned()
                wind_n = smoother_n.update(raw_n)
                wind_e = smoother_e.update(raw_e)
                wind_n, wind_e = apply_deadband(wind_n, wind_e, WIND_DEADBAND_MPS)
                accel_n = max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, -ACCEL_GAIN * wind_n))
                accel_e = max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, -ACCEL_GAIN * wind_e))
                current_cmd["accel_n"] = accel_n
                current_cmd["accel_e"] = accel_e
            else:
                current_cmd["accel_n"] = 0.0
                current_cmd["accel_e"] = 0.0

            pos_error = ((north - nominal["north"]) ** 2 + (east - nominal["east"]) ** 2) ** 0.5
            peak_error = max(peak_error, pos_error)

            writer.writerow([
                wind_label, phase_name, f"{t:.2f}", f"{true_vy:.3f}", f"{true_vx:.3f}",
                f"{wind_n:.2f}", f"{wind_e:.2f}",
                f"{current_cmd['accel_n']:.3f}", f"{current_cmd['accel_e']:.3f}",
                f"{north:.3f}", f"{east:.3f}", f"{pos_error:.3f}",
                f"{roll:.2f}", f"{pitch:.2f}",
            ])

            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        return peak_error

    results = []
    for cond in GUST_CONDITIONS:
        wind_label, base_vx, base_vy, amp_fraction, period_s = cond
        base_speed = math.hypot(base_vx, base_vy)
        print(f"\n{'='*70}\n=== gust 조건: {wind_label} "
              f"(base={base_speed:.1f}m/s, amp={amp_fraction*100:.0f}%, period={period_s:.0f}s) "
              f"===\n{'='*70}")

        print("  -- 보정 OFF --")
        await set_wind(0.0, 0.0)
        await asyncio.sleep(CALM_SETTLE_S)
        nominal["north"] = latest_pv["north"]
        nominal["east"] = latest_pv["east"]
        peak_off = await run_trial(wind_label, "correction_off", False, cond)
        print(f"    peak_pos_error(OFF) = {peak_off:.3f}m")

        await set_wind(0.0, 0.0)
        current_cmd["accel_n"] = 0.0
        current_cmd["accel_e"] = 0.0
        await asyncio.sleep(PHASE_GAP_S)

        print("  -- 보정 ON --")
        await asyncio.sleep(CALM_SETTLE_S)
        nominal["north"] = latest_pv["north"]
        nominal["east"] = latest_pv["east"]
        peak_on = await run_trial(wind_label, "correction_on", True, cond)
        print(f"    peak_pos_error(ON)  = {peak_on:.3f}m")

        improvement = (peak_off - peak_on) / peak_off * 100 if peak_off > 1e-6 else 0.0
        print(f"    개선율 = {improvement:+.1f}%")
        results.append((wind_label, base_speed, peak_off, peak_on, improvement))

        await set_wind(0.0, 0.0)
        current_cmd["accel_n"] = 0.0
        current_cmd["accel_e"] = 0.0
        await asyncio.sleep(PHASE_GAP_S)

    csv_file.close()

    print(f"\n{'='*70}\n=== 전체 요약 (gust 조건) ===\n{'='*70}")
    print(f"{'조건':16s} {'base(m/s)':>10s} {'OFF peak':>10s} {'ON peak':>10s} {'개선율':>8s}")
    for label, base_speed, off, on, imp in results:
        print(f"{label:16s} {base_speed:10.2f} {off:10.3f} {on:10.3f} {imp:+7.1f}%")

    n_improved = sum(1 for r in results if r[4] > 0)
    print(f"\n개선된 조건: {n_improved}/{len(results)}")
    print(f"CSV 저장됨: {csv_path}")

    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > 1.5 * SEND_PERIOD_S)
        print(
            f"\n[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  임계치 초과={n_over}/{len(send_gaps)}"
        )

    await set_wind(5.0, 2.0)

    for task in (sender_task, pv_task, att_task, gyro_task):
        task.cancel()
    for task in (sender_task, pv_task, att_task, gyro_task):
        try:
            await task
        except asyncio.CancelledError:
            pass

    print("\n--- Offboard 종료 ---")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard 종료 실패: {error._result.result}")

    print("\n--- Landing ---")
    await drone.action.land()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-> 착륙 완료 및 디스암 확인")
            break

    print("\nPINN 보정 gust 스윕 테스트 완료.")


if __name__ == "__main__":
    asyncio.run(run())
