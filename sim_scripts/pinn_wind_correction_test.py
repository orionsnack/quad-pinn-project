"""
학습된 PINN 바람 추정 모델을 실제 비행에 연결해서,
"PID-only(보정 없음)" vs "PINN 가속도 피드포워드 보정 ON"을 같은 바람 조건에서 비교.

--- 이전 버전(position 보정)에서 겪은 문제와 이번 설계 ---
처음엔 추정된 바람만큼 position setpoint(목표 위치)를 바람 반대방향으로 옮기는
방식을 썼는데, 두 가지 문제가 있었음:
  1) setpoint가 계속 움직이니 "그 setpoint를 못 따라가서 생긴 지연"까지 위치오차로
     잡혀 모델 입력을 오염시켜 폭주(발산)함
  2) 애초에 PID가 이미 자세(roll/pitch) 트림만으로 정상상태 위치오차를 5cm 이내로
     거의 완벽히 없애고 있어서, 위치를 흔드는 보정은 도움될 여지가 별로 없었음
     (오히려 새 과도응답만 계속 만들어 냄)

이번 버전은 MAVSDK의 set_position_velocity_acceleration_ned를 사용:
  - position setpoint는 처음 고정한 값 그대로 절대 안 움직임 (목표는 안 흔들림)
  - 대신 추정된 바람과 반대방향의 "가속도(feedforward)"만 추가로 실어서 보냄
  - PX4는 이 가속도를 자기 위치제어 출력에 더해서 사용 -> "어디로 갈지"는 안 헷갈리고
    "가는 길을 옆에서 밀어주는" 역할만 함 -> 이전 폭주 문제가 구조적으로 생기지 않음

두 phase를 동일한 순서로 반복 (무풍 -> 강풍 온셋 -> 15초 관찰):
  Phase A: 보정 OFF (PID-only baseline)
  Phase B: 보정 ON  (PINN 추정치 기반 가속도 피드포워드)

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import asyncio
import csv
import datetime
import sys
import time
from pathlib import Path

import torch
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw, AccelerationNed)

sys.path.insert(0, str(Path(__file__).parent.parent / "offline_training"))
from train_wind_estimator import WindPINN, WINDOW, FEATURES, yaw_decompose  # noqa: E402


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
WIND_VX_MPS = 8.0   # "strong" 조건 (wind_sweep_baseline.py와 동일)
WIND_VY_MPS = 3.0
ACCEL_GAIN = 0.15       # 추정 바람(m/s) -> 가속도 피드포워드(m/s^2) 변환 게인.
                         # 참고: roll/pitch 몇도 트림으로 강풍(8~9m/s)을 버틸 때 실제
                         # 필요한 가속도는 대략 0.5~3 m/s^2 수준(g*tan(tilt)) 이었음.
                         # 이 게인은 그 범위를 크게 못 벗어나도록 보수적으로 잡은 값.
MAX_ACCEL_MPS2 = 2.0    # 안전장치: 모델이 오추정해도 과도한 가속도가 나가지 않도록 clip.
WIND_DEADBAND_MPS = 1.0  # 무풍(calm)일 때 추정 잡음만으로 보정이 살짝 손해를 보는 문제
                          # 완화용 (다중조건 스윕에서 calm만 -67.5%로 유일하게 나빠졌었음).
                          # speed<=DEADBAND면 보정 0, 그 이상은 부드럽게(선형) 커짐 -
                          # 문턱값에서 뚝 끊기지 않게 방향은 유지한 채 크기만 깎음.
TRIAL_DURATION_S = 15.0
CALM_SETTLE_S = 3.0     # 무풍 상태로 되돌려 안정화하는 시간
PHASE_GAP_S = 2.0
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.05   # 학습 데이터와 동일한 샘플링 간격 (모델 입력 분포 일치)
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

MODEL_PATH = Path(__file__).parent.parent / "offline_training" / "wind_estimator.pt"


async def set_wind(vx, vy, vz=0.0):
    proc = await asyncio.create_subprocess_exec(
        "gz", "topic", "-t", f"/world/{WORLD_NAME}/wind",
        "-m", "gz.msgs.Wind",
        "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
    )
    await proc.wait()


class WindCorrector:
    """학습된 PINN으로 최근 상태 윈도우 -> 바람 추정."""

    def __init__(self, model_path):
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        self.model = WindPINN(ckpt["window"], len(ckpt["features"]))
        self.model.load_state_dict(ckpt["model_state"])
        self.model.eval()
        self.X_mean = torch.tensor(ckpt["X_mean"], dtype=torch.float32)
        self.X_std = torch.tensor(ckpt["X_std"], dtype=torch.float32)
        self.window = ckpt["window"]
        self.features = ckpt["features"]
        self.buffer = []  # 최근 self.window개의 feature 벡터

    def push_state(self, feat_vec):
        self.buffer.append(feat_vec)
        if len(self.buffer) > self.window:
            self.buffer.pop(0)

    def ready(self):
        return len(self.buffer) == self.window

    @torch.no_grad()
    def estimate_wind_ned(self):
        """반환: (wind_north, wind_east) - ENU->NED 변환까지 완료된 추정치."""
        x = torch.tensor(self.buffer, dtype=torch.float32).flatten()
        xn = (x - self.X_mean) / self.X_std
        wind_enu = self.model(xn.unsqueeze(0)).squeeze(0)  # [vx_enu(East), vy_enu(North)]
        wind_north = wind_enu[1].item()
        wind_east = wind_enu[0].item()
        return wind_north, wind_east


class EmaSmoother:
    """추정치가 매 스텝(0.05s)마다 크게 흔들리는 걸 완화. 실제 바람은 거의
    일정/완만하게 변하므로, 지수이동평균으로 고주파 노이즈만 걸러냄."""

    def __init__(self, time_constant_s, dt_s):
        self.alpha = dt_s / (time_constant_s + dt_s)
        self.value = None

    def update(self, x):
        if self.value is None:
            self.value = x
        else:
            self.value = self.alpha * x + (1 - self.alpha) * self.value
        return self.value


def apply_deadband(wind_n, wind_e, deadband):
    """speed<=deadband면 (0,0), 그 이상이면 방향은 유지한 채 크기만
    (speed-deadband)만큼으로 줄여서 반환. 문턱값 근처에서 뚝 끊기지 않고
    연속적으로 이어짐 (speed==deadband에서 정확히 0, 커질수록 원래 벡터에 수렴)."""
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

    # --- 모니터링 백그라운드 태스크 ---
    latest_pv = {"north": None, "east": None, "down": None,
                 "vn": None, "ve": None, "vd": None}

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

    # --- Offboard 송신 전담 태스크 ---
    # position/velocity는 항상 고정 nominal(호버 지점, 0속도) 그대로 보내고,
    # accel_n/accel_e만 보정에 따라 매 사이클 바뀜 - target 자체는 절대 안 흔들림.
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
    csv_path = f"../logs/pinn_correction_ab_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "phase", "t_s", "wind_est_north", "wind_est_east",
        "accel_n_m_s2", "accel_e_m_s2",
        "actual_north_m", "actual_east_m", "pos_error_m",
        "roll_deg", "pitch_deg",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    async def run_trial(phase_name, use_correction):
        corrector.buffer.clear()
        smoother_n = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        smoother_e = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
        next_log = time.monotonic()
        peak_error = 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            north, east = latest_pv["north"], latest_pv["east"]
            vn, ve = latest_pv["vn"], latest_pv["ve"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]

            # pos_err(위치오차)는 feature에 안 씀 - 이전 버전에서 폐루프 폭주의
            # 원인이었음 (train_wind_estimator.py 주석 참고). 이번엔 position
            # setpoint 자체가 안 움직이므로 위험은 줄었지만, 동일 모델을 그대로
            # 재사용하기 위해 feature 구성도 학습 때와 동일하게 유지.
            r_cos, r_sin, p_cos, p_sin = yaw_decompose(roll, pitch, yaw)
            feat = {"vn_m_s": vn, "ve_m_s": ve, "roll_cos_yaw": r_cos, "roll_sin_yaw": r_sin,
                    "pitch_cos_yaw": p_cos, "pitch_sin_yaw": p_sin}
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

            pos_error = ((north - nominal["north"]) ** 2
                         + (east - nominal["east"]) ** 2) ** 0.5
            peak_error = max(peak_error, pos_error)

            writer.writerow([
                phase_name, f"{t:.2f}", f"{wind_n:.2f}", f"{wind_e:.2f}",
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

        print(f"  [{phase_name}] peak_pos_error={peak_error:.3f}m  "
              f"final_pos_error={pos_error:.3f}m  final_roll={roll:.1f} pitch={pitch:.1f}")
        return peak_error

    # --- Phase A: 보정 OFF ---
    print(f"\n=== Phase A: PID-only baseline (보정 OFF), 바람 온셋 관찰 ===")
    await set_wind(0.0, 0.0)
    await asyncio.sleep(CALM_SETTLE_S)
    nominal["north"] = latest_pv["north"]
    nominal["east"] = latest_pv["east"]
    await set_wind(WIND_VX_MPS, WIND_VY_MPS)
    peak_off = await run_trial("correction_off", use_correction=False)

    print(f"\n-- 정지 {PHASE_GAP_S:.0f}초 --")
    await set_wind(0.0, 0.0)
    current_cmd["accel_n"] = 0.0
    current_cmd["accel_e"] = 0.0
    await asyncio.sleep(PHASE_GAP_S)

    # --- Phase B: 보정 ON ---
    print(f"\n=== Phase B: PINN 가속도 보정 ON, 동일 바람 온셋 관찰 ===")
    await asyncio.sleep(CALM_SETTLE_S)
    nominal["north"] = latest_pv["north"]
    nominal["east"] = latest_pv["east"]
    await set_wind(WIND_VX_MPS, WIND_VY_MPS)
    peak_on = await run_trial("correction_on", use_correction=True)

    csv_file.close()
    print(f"\n=== 비교 결과 ===")
    print(f"  보정 OFF 피크 오차: {peak_off:.3f}m")
    print(f"  보정 ON  피크 오차: {peak_on:.3f}m")
    improvement = (peak_off - peak_on) / peak_off * 100 if peak_off > 0 else 0.0
    print(f"  변화: {improvement:+.1f}% ({'개선' if improvement > 0 else '악화'})")
    print(f"CSV 저장됨: {csv_path}")

    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > 1.5 * SEND_PERIOD_S)
        print(
            f"[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  임계치 초과={n_over}/{len(send_gaps)}"
        )

    await set_wind(5.0, 2.0)  # 기본값 복원

    for task in (sender_task, pv_task, att_task):
        task.cancel()
    for task in (sender_task, pv_task, att_task):
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

    print("\nPINN 보정 A/B 테스트 완료.")


if __name__ == "__main__":
    asyncio.run(run())
