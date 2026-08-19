"""
pinn_wind_correction_test.py(강풍 1개 조건)의 다중 조건 버전.
여러 바람 조건(calm/light/default/strong/crosswind) 각각에 대해 "보정 OFF" ->
"보정 ON"을 순서대로 돌려서, PINN 가속도 피드포워드 보정의 효과가 특정 조건에서만
우연히 좋았던 게 아니라 여러 조건에서 일관되게 나타나는지 확인.

한 번의 이륙-착륙 세션 안에서 전부 처리. 조건마다:
  무풍 안정화 -> 바람 온셋 -> 보정 OFF로 15초 관찰 -> 무풍으로 복귀 -> 무풍 안정화
  -> 같은 바람 다시 온셋 -> 보정 ON으로 15초 관찰
끝나면 조건별 peak_pos_error(OFF vs ON)와 개선율을 표로 요약.

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import argparse
import asyncio
import csv
import datetime
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
WIND_CONDITIONS = [
    ("calm", 0.0, 0.0),
    ("light", 2.0, 1.0),
    ("default", 5.0, 2.0),
    ("strong", 8.0, 3.0),
    ("crosswind", 0.0, 6.0),
]
ACCEL_GAIN = 0.15  # 체계적 튜닝 스윕에서 0.20/deadband 조정을 시도했으나 전체 조건
MAX_ACCEL_MPS2 = 2.0    # 기준으로는 원래 값이 더 균형 잡힌 것으로 확인됨 (12-8절 참고)
WIND_DEADBAND_MPS = 1.0  # calm에서 추정 잡음만으로 보정이 살짝 손해보던 문제 완화용

# 적응형 게인 (2026-08-17 추가, EXPERIMENTS.md 12-8/12-20절): 12-8절에서 단일
# ACCEL_GAIN 상수로는 "강풍 개선하려고 올리면 무풍/약풍이 나빠지는" 트레이드오프가
# 확인됨 - 풍속에 비례해 게인을 선형으로 키우면 양쪽을 동시에 잡을 수 있는지 검증.
# CLI --adaptive로 켜기 전엔 기존처럼 ACCEL_GAIN 고정값 그대로 사용(기본 동작 불변).
USE_ADAPTIVE_GAIN = "--adaptive" in sys.argv
ACCEL_GAIN_MIN = 0.10   # deadband 통과 직후(약한 신호)엔 낮게 - 추정 잡음 과대반응 억제
ACCEL_GAIN_MAX = 0.25   # 강풍에선 높게 - 12-8절 스윕 범위(0.05~0.30) 상단 근처
ADAPTIVE_GAIN_SPEED_LOW = WIND_DEADBAND_MPS   # 이 풍속 이하는 ACCEL_GAIN_MIN
ADAPTIVE_GAIN_SPEED_HIGH = 8.0                # strong 조건 풍속 - 이 이상은 ACCEL_GAIN_MAX


def adaptive_gain(speed_est):
    t = (speed_est - ADAPTIVE_GAIN_SPEED_LOW) / (ADAPTIVE_GAIN_SPEED_HIGH - ADAPTIVE_GAIN_SPEED_LOW)
    t = max(0.0, min(1.0, t))
    return ACCEL_GAIN_MIN + (ACCEL_GAIN_MAX - ACCEL_GAIN_MIN) * t
TRIAL_DURATION_S = 15.0
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
    if USE_ADAPTIVE_GAIN:
        print(f"  게인 모드: 적응형 (min={ACCEL_GAIN_MIN}, max={ACCEL_GAIN_MAX}, "
              f"{ADAPTIVE_GAIN_SPEED_LOW}~{ADAPTIVE_GAIN_SPEED_HIGH}m/s 구간 선형)")
    else:
        print(f"  게인 모드: 고정 (ACCEL_GAIN={ACCEL_GAIN})")

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
    csv_path = f"../../logs/pinn_correction_sweep_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "wind_label", "phase", "t_s", "wind_est_north", "wind_est_east",
        "accel_n_m_s2", "accel_e_m_s2",
        "actual_north_m", "actual_east_m", "pos_error_m",
        "roll_deg", "pitch_deg",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    async def run_trial(wind_label, phase_name, use_correction):
        corrector.buffer.clear()
        smoother_n = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        smoother_e = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        # 적응형 게인 자체도 별도로 스무딩(1.0초, 풍속 스무더보다 느림) - 게인을
        # 풍속 추정치의 함수로 만들면 추정 잡음이 게인 변동을 통해 2차로 명령에
        # 전달되어 오히려 더 떨리는 문제를 실측으로 확인함(EXPERIMENTS.md 12-20절).
        gain_smoother = EmaSmoother(time_constant_s=1.0, dt_s=LOG_INTERVAL_S)
        n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
        next_log = time.monotonic()
        peak_error = 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
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
                if USE_ADAPTIVE_GAIN:
                    raw_gain = adaptive_gain((wind_n ** 2 + wind_e ** 2) ** 0.5)
                    gain = gain_smoother.update(raw_gain)
                else:
                    gain = ACCEL_GAIN
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
                wind_label, phase_name, f"{t:.2f}", f"{wind_n:.2f}", f"{wind_e:.2f}",
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
    for wind_label, wind_vx, wind_vy in WIND_CONDITIONS:
        print(f"\n{'='*60}\n=== 바람 조건: {wind_label} (vx={wind_vx}, vy={wind_vy}) ===\n{'='*60}")

        print("  -- 보정 OFF --")
        await set_wind(0.0, 0.0)
        await asyncio.sleep(CALM_SETTLE_S)
        nominal["north"] = latest_pv["north"]
        nominal["east"] = latest_pv["east"]
        await set_wind(wind_vx, wind_vy)
        peak_off = await run_trial(wind_label, "correction_off", use_correction=False)
        print(f"    peak_pos_error(OFF) = {peak_off:.3f}m")

        await set_wind(0.0, 0.0)
        current_cmd["accel_n"] = 0.0
        current_cmd["accel_e"] = 0.0
        await asyncio.sleep(PHASE_GAP_S)

        print("  -- 보정 ON --")
        await asyncio.sleep(CALM_SETTLE_S)
        nominal["north"] = latest_pv["north"]
        nominal["east"] = latest_pv["east"]
        await set_wind(wind_vx, wind_vy)
        peak_on = await run_trial(wind_label, "correction_on", use_correction=True)
        print(f"    peak_pos_error(ON)  = {peak_on:.3f}m")

        improvement = (peak_off - peak_on) / peak_off * 100 if peak_off > 1e-6 else 0.0
        print(f"    개선율 = {improvement:+.1f}%")
        results.append((wind_label, wind_vx, wind_vy, peak_off, peak_on, improvement))

        await set_wind(0.0, 0.0)
        current_cmd["accel_n"] = 0.0
        current_cmd["accel_e"] = 0.0
        await asyncio.sleep(PHASE_GAP_S)

    csv_file.close()

    print(f"\n{'='*70}\n=== 전체 요약 ===\n{'='*70}")
    print(f"{'조건':10s} {'풍속(m/s)':>10s} {'OFF peak':>10s} {'ON peak':>10s} {'개선율':>8s}")
    for label, vx, vy, off, on, imp in results:
        speed = (vx**2 + vy**2) ** 0.5
        print(f"{label:10s} {speed:10.2f} {off:10.3f} {on:10.3f} {imp:+7.1f}%")

    n_improved = sum(1 for r in results if r[5] > 0)
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

    print("\nPINN 보정 스윕 테스트 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--calm-settle-s", type=float, default=CALM_SETTLE_S,
                         help="조건 전환 시 무풍 안정화 대기 시간(초) - 12-22절 분산 원인 실험용")
    parser.add_argument("--phase-gap-s", type=float, default=PHASE_GAP_S,
                         help="OFF/ON phase 사이 대기 시간(초) - 12-22절 분산 원인 실험용")
    args = parser.parse_args()
    CALM_SETTLE_S = args.calm_settle_s
    PHASE_GAP_S = args.phase_gap_s
    asyncio.run(run())
