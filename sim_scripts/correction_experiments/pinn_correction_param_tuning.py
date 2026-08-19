"""
PINN 가속도 피드포워드 보정의 두 하이퍼파라미터(`ACCEL_GAIN`, `WIND_DEADBAND_MPS`)를
체계적으로 스윕하는 스크립트. 지금까지 두 값 다 손으로 정한 값(0.15 / 1.0)이었고,
gust A/B 검증(pinn_wind_correction_gust_sweep.py)에서 약한 gust(light_gust)만
deadband로 못 걸러진 잡음 때문에 손해를 봤음 (EXPERIMENTS.md 12-7절).

모든 조건 x 모든 파라미터 조합을 다 돌리면 시간이 너무 오래 걸리므로, 각 파라미터가
가장 민감하게 반응할 대표 조건만 골라서 one-factor-at-a-time 방식으로 스윕:
  - deadband 스윕: light(고정) + light_gust — 문제가 드러난 "약한 바람" 대역.
    ACCEL_GAIN은 현재 기본값(0.15)로 고정.
  - gain 스윕: default(고정) + strong(고정) — 보정 효과가 크게 나타나는 대역이라
    gain을 올렸을 때 개선폭이 더 커지는지, 너무 키우면 roll/pitch가 흔들리기
    시작하는지 확인. WIND_DEADBAND_MPS는 현재 기본값(1.0)로 고정.

각 조건마다 OFF 트라이얼은 파라미터에 안 좌우되므로(보정 자체가 꺼져 있음) 1번만
측정하고, 그 baseline 대비 각 파라미터 값에서의 ON 트라이얼들을 비교함.

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

# (label, base_vx, base_vy, amp_fraction, period_s) - amp_fraction=0이면 고정바람
DEADBAND_PROBE_CONDITIONS = [
    ("light_fixed", 2.0, 1.0, 0.0, 1.0),
    ("light_gust", 2.0, 1.0, 0.6, 6.0),
]
GAIN_PROBE_CONDITIONS = [
    ("default_fixed", 5.0, 2.0, 0.0, 1.0),
    ("strong_fixed", 8.0, 3.0, 0.0, 1.0),
]

DEADBAND_VALUES = [0.5, 1.0, 1.5, 2.0]
ACCEL_GAIN_VALUES = [0.05, 0.10, 0.15, 0.20, 0.30]

ACCEL_GAIN_DEFAULT = 0.15   # deadband 스윕 중 고정
WIND_DEADBAND_DEFAULT = 1.0  # gain 스윕 중 고정
MAX_ACCEL_MPS2 = 2.0

TRIAL_DURATION_S = 18.0
CALM_SETTLE_S = 3.0
PHASE_GAP_S = 2.0
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ
GUST_UPDATE_INTERVAL_S = 1.0

MODEL_PATH = Path(__file__).parent.parent.parent / "offline_training" / "wind_estimator.pt"


async def set_wind(vx, vy, vz=0.0):
    proc = await asyncio.create_subprocess_exec(
        "gz", "topic", "-t", f"/world/{WORLD_NAME}/wind",
        "-m", "gz.msgs.Wind",
        "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
    )
    await proc.wait()
    if proc.returncode != 0:
        # 조용히 넘기면 바람이 실제로 안 걸린 채 트라이얼이 진행될 수 있음 - 실제로
        # 이걸로 엉터리 결과가 나온 적 있어(EXPERIMENTS.md 12-17/12-21절) 예외로 드러냄.
        raise RuntimeError(f"gz topic pub 실패 (returncode={proc.returncode})")


def wind_at(cond, t):
    """cond: (label, base_vx, base_vy, amp_fraction, period_s). amp_fraction=0이면
    고정바람(방향/크기 불변), 아니면 크기만 사인파로 진동(방향 고정)."""
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
    csv_path = f"../../logs/pinn_correction_param_tuning_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "sweep_phase", "wind_label", "param_name", "param_value", "trial_phase", "t_s",
        "wind_est_north", "wind_est_east", "accel_n_m_s2", "accel_e_m_s2",
        "actual_north_m", "actual_east_m", "pos_error_m", "roll_deg", "pitch_deg",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    updates_per_log = max(1, round(GUST_UPDATE_INTERVAL_S / LOG_INTERVAL_S))

    async def run_trial(sweep_phase, wind_label, param_name, param_value,
                         trial_phase, use_correction, cond, deadband, gain):
        corrector.buffer.clear()
        smoother_n = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        smoother_e = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
        next_log = time.monotonic()
        peak_error = 0.0
        max_roll, max_pitch = 0.0, 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            true_vx, true_vy = wind_at(cond, t)
            if i % updates_per_log == 0:
                await set_wind(true_vx, true_vy)

            north, east = latest_pv["north"], latest_pv["east"]
            vn, ve = latest_pv["vn"], latest_pv["ve"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]
            max_roll = max(max_roll, abs(roll))
            max_pitch = max(max_pitch, abs(pitch))

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
                wind_n, wind_e = apply_deadband(wind_n, wind_e, deadband)
                accel_n = max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, -gain * wind_n))
                accel_e = max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, -gain * wind_e))
                current_cmd["accel_n"] = accel_n
                current_cmd["accel_e"] = accel_e
            else:
                current_cmd["accel_n"] = 0.0
                current_cmd["accel_e"] = 0.0

            pos_error = ((north - nominal["north"]) ** 2 + (east - nominal["east"]) ** 2) ** 0.5
            peak_error = max(peak_error, pos_error)

            writer.writerow([
                sweep_phase, wind_label, param_name, param_value, trial_phase, f"{t:.2f}",
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

        return peak_error, max_roll, max_pitch

    async def settle_and_capture_nominal():
        await set_wind(0.0, 0.0)
        current_cmd["accel_n"] = 0.0
        current_cmd["accel_e"] = 0.0
        await asyncio.sleep(CALM_SETTLE_S)
        nominal["north"] = latest_pv["north"]
        nominal["east"] = latest_pv["east"]

    deadband_results = []
    print(f"\n{'#'*70}\n# 1부: WIND_DEADBAND_MPS 스윕 (ACCEL_GAIN={ACCEL_GAIN_DEFAULT} 고정)\n{'#'*70}")
    for cond in DEADBAND_PROBE_CONDITIONS:
        label = cond[0]
        print(f"\n=== 조건: {label} ===")

        await settle_and_capture_nominal()
        peak_off, _, _ = await run_trial("deadband", label, "baseline_off", 0.0,
                                          "off", False, cond, WIND_DEADBAND_DEFAULT, ACCEL_GAIN_DEFAULT)
        print(f"  OFF baseline peak = {peak_off:.3f}m")
        await asyncio.sleep(PHASE_GAP_S)

        for db in DEADBAND_VALUES:
            await settle_and_capture_nominal()
            peak_on, max_roll, max_pitch = await run_trial(
                "deadband", label, "deadband_mps", db, "on", True, cond, db, ACCEL_GAIN_DEFAULT)
            improvement = (peak_off - peak_on) / peak_off * 100 if peak_off > 1e-6 else 0.0
            print(f"  deadband={db:.1f}  peak_ON={peak_on:.3f}m  개선율={improvement:+.1f}%  "
                  f"max|roll|={max_roll:.1f} max|pitch|={max_pitch:.1f}")
            deadband_results.append((label, db, peak_off, peak_on, improvement, max_roll, max_pitch))
            await asyncio.sleep(PHASE_GAP_S)

    gain_results = []
    print(f"\n{'#'*70}\n# 2부: ACCEL_GAIN 스윕 (WIND_DEADBAND_MPS={WIND_DEADBAND_DEFAULT} 고정)\n{'#'*70}")
    for cond in GAIN_PROBE_CONDITIONS:
        label = cond[0]
        print(f"\n=== 조건: {label} ===")

        await settle_and_capture_nominal()
        peak_off, _, _ = await run_trial("gain", label, "baseline_off", 0.0,
                                          "off", False, cond, WIND_DEADBAND_DEFAULT, ACCEL_GAIN_DEFAULT)
        print(f"  OFF baseline peak = {peak_off:.3f}m")
        await asyncio.sleep(PHASE_GAP_S)

        for gain in ACCEL_GAIN_VALUES:
            await settle_and_capture_nominal()
            peak_on, max_roll, max_pitch = await run_trial(
                "gain", label, "accel_gain", gain, "on", True, cond, WIND_DEADBAND_DEFAULT, gain)
            improvement = (peak_off - peak_on) / peak_off * 100 if peak_off > 1e-6 else 0.0
            print(f"  gain={gain:.2f}  peak_ON={peak_on:.3f}m  개선율={improvement:+.1f}%  "
                  f"max|roll|={max_roll:.1f} max|pitch|={max_pitch:.1f}")
            gain_results.append((label, gain, peak_off, peak_on, improvement, max_roll, max_pitch))
            await asyncio.sleep(PHASE_GAP_S)

    csv_file.close()

    print(f"\n{'='*78}\n=== 요약 1: WIND_DEADBAND_MPS 스윕 ===\n{'='*78}")
    print(f"{'조건':14s} {'deadband':>9s} {'OFF peak':>9s} {'ON peak':>9s} {'개선율':>8s} "
          f"{'max|roll|':>10s} {'max|pitch|':>11s}")
    for label, db, off, on, imp, mr, mp in deadband_results:
        print(f"{label:14s} {db:9.1f} {off:9.3f} {on:9.3f} {imp:+7.1f}% {mr:10.1f} {mp:11.1f}")

    print(f"\n{'='*78}\n=== 요약 2: ACCEL_GAIN 스윕 ===\n{'='*78}")
    print(f"{'조건':14s} {'gain':>9s} {'OFF peak':>9s} {'ON peak':>9s} {'개선율':>8s} "
          f"{'max|roll|':>10s} {'max|pitch|':>11s}")
    for label, gain, off, on, imp, mr, mp in gain_results:
        print(f"{label:14s} {gain:9.2f} {off:9.3f} {on:9.3f} {imp:+7.1f}% {mr:10.1f} {mp:11.1f}")

    print(f"\nCSV 저장됨: {csv_path}")

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

    print("\nPINN 보정 파라미터 튜닝 스윕 완료.")


if __name__ == "__main__":
    asyncio.run(run())
