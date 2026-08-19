"""
PINN이 추정한 회전 방해토크(tau_disturbance)를 PX4 rate controller에 실제
피드포워드로 넣었을 때 자세(roll/pitch) 흔들림이 줄어드는지 검증하는 A/B 테스트.

병진 보정(pinn_wind_correction_sweep.py)과 달리 이건 위치가 아니라 자세 안정성이
목표이므로, 위치는 그대로 offboard 위치제어로 고정해두고(병진 보정은 끈 채) roll/pitch
피크 편차만 비교한다.

주입 경로: PX4 firmware(mc_rate_control.cpp)에 새로 추가한 MAVLink
DEBUG_VECT("TAU_FF", x, y, z) 채널 - torque_setpoint([-1,1] 정규화)에 직접 더해짐.
tau_disturbance는 PINN이 N*m 단위로 추정하므로, TAU_FF_GAIN으로 정규화 스케일로
환산한다(1차 근사치, 물리적으로 정밀 캘리브레이션한 값 아님 - 0.05~0.15 정규화
범위에서 실제 roll rate가 선형·안정적으로 반응하는 것만 확인된 상태).

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함, 그리고 firmware가
tau_disturbance 피드포워드 패치(mc_rate_control.cpp/hpp)로 재빌드되어 있어야 함.
"""
import asyncio
import csv
import datetime
import sys
import time
from pathlib import Path

import torch
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw, AccelerationNed
from mavsdk.mavlink_direct import MavlinkMessage

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "offline_training"))
from wind_pinn_model import WindPINN, FEATURES, yaw_decompose  # noqa: E402

WORLD_NAME = "windy"
# 병진 A/B(pinn_wind_correction_sweep.py)와 동일한 표준 5조건 - 회전 쪽 초기
# 검증(12-16/12-17절)은 3조건(calm/default/strong)만 썼으므로, gain=0.2가 더
# 넓은 조건에서도 유효한지 보려면 light/crosswind까지 포함해야 함.
ALL_WIND_CONDITIONS = [
    ("calm", 0.0, 0.0),
    ("light", 2.0, 1.0),
    ("default", 5.0, 2.0),
    ("strong", 8.0, 3.0),
    ("crosswind", 0.0, 6.0),
]
# CLI: python pinn_rotation_correction_test.py [gain] [condition_label] [target_yaw_deg]
# gain 기본값 0.2 (2026-08-16 스윕으로 확정 - EXPERIMENTS.md 12-17절): 1.0에서는
# 3조건 다 손해/중립이었는데, 0.2로 낮추니 default +2.8%/+3.8%(재현됨), strong
# +3.2%로 첫 순이익 확인. calm은 -4.5%지만 절대오차 0.036도로 노이즈 수준.
# target_yaw_deg 생략 시 SITL 스폰 시 yaw 그대로 사용 (지금까지 전부 이렇게
# 테스트함) - 각속도(wx/wy/wz)는 body frame이라 이론상 yaw 종속성이 없어야
# 하지만(wind_pinn_model.py 참고), 병진 쪽에서 실제로 yaw 의존성 버그가 있었던
# 전례(12-10절)가 있어 다른 yaw에서도 gain=0.2가 유효한지 확인할 가치가 있음.
TAU_FF_GAIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
_cond_label = sys.argv[2] if len(sys.argv) > 2 else None
TARGET_YAW_DEG = float(sys.argv[3]) if len(sys.argv) > 3 else None
WIND_CONDITIONS = (
    [c for c in ALL_WIND_CONDITIONS if c[0] == _cond_label] if _cond_label else ALL_WIND_CONDITIONS
)
YAW_TOLERANCE_DEG = 3.0
YAW_SETTLE_TIMEOUT_S = 10.0


def yaw_diff_deg(a, b):
    d = (a - b + 180) % 360 - 180
    return abs(d)
MAX_TAU_FF = 0.25   # 안전 클램프 (캘리브레이션 테스트한 0.15보다 여유 있게)
TRIAL_DURATION_S = 15.0
CALM_SETTLE_S = 5.0   # 12-23절: 3.0으로는 반복측정 시 분산이 극심함(최대 20배) 확인, 5.0으로 상향
PHASE_GAP_S = 4.0     # 12-23절: 2.0으로는 반복측정 시 분산이 극심함 확인, 4.0으로 상향
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ
TAUFF_RATE_HZ = 20.0
TAUFF_PERIOD_S = 1.0 / TAUFF_RATE_HZ

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


class RotationCorrector:
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
    def estimate_tau_dist(self):
        x = torch.tensor(self.buffer, dtype=torch.float32).flatten()
        xn = (x - self.X_mean) / self.X_std
        pred = self.model(xn.unsqueeze(0)).squeeze(0)
        return pred[2].item(), pred[3].item(), pred[4].item()  # tau_x, tau_y, tau_z (N*m)


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
    corrector = RotationCorrector(MODEL_PATH)
    print(f"  window={corrector.window}  features={corrector.features}")

    await set_wind(0.0, 0.0)

    print("\n--- Arming ---")
    await drone.action.arm()
    print("\n--- Takeoff ---")
    await drone.action.takeoff()

    t_start = time.monotonic()
    reached_altitude = False
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= SAFE_ALTITUDE_M:
            print(f"  -> 안전 고도 도달 ({position.relative_altitude_m:.2f}m)")
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
        async for a in drone.telemetry.attitude_euler():
            latest_att["roll"] = a.roll_deg
            latest_att["pitch"] = a.pitch_deg
            latest_att["yaw"] = a.yaw_deg

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

    async def offboard_sender():
        while True:
            try:
                await drone.offboard.set_position_velocity_acceleration_ned(
                    PositionNedYaw(nominal["north"], nominal["east"], nominal["down"], nominal["yaw"]),
                    VelocityNedYaw(0.0, 0.0, 0.0, nominal["yaw"]),
                    AccelerationNed(0.0, 0.0, 0.0),
                )
            except Exception as exc:
                print(f"  [!!! offboard_sender 예외] {exc}")
            await asyncio.sleep(SEND_PERIOD_S)

    print("\n--- Offboard 진입 준비 ---")
    await drone.offboard.set_position_velocity_acceleration_ned(
        PositionNedYaw(nominal["north"], nominal["east"], nominal["down"], nominal["yaw"]),
        VelocityNedYaw(0.0, 0.0, 0.0, nominal["yaw"]),
        AccelerationNed(0.0, 0.0, 0.0),
    )
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    sender_task = asyncio.create_task(offboard_sender())

    if TARGET_YAW_DEG is not None:
        print(f"\n--- 목표 yaw {TARGET_YAW_DEG:.0f}도로 회전 중 (현재 {latest_att['yaw']:.1f}도) ---")
        nominal["yaw"] = TARGET_YAW_DEG
        t_yaw_start = time.monotonic()
        while yaw_diff_deg(latest_att["yaw"], TARGET_YAW_DEG) > YAW_TOLERANCE_DEG:
            if time.monotonic() - t_yaw_start > YAW_SETTLE_TIMEOUT_S:
                print(f"  [경고] {YAW_SETTLE_TIMEOUT_S:.0f}초 내에 목표 yaw 미도달 - 그냥 진행")
                break
            await asyncio.sleep(0.1)
        print(f"  -> yaw {latest_att['yaw']:.1f}도로 안정화")

    current_tauff = {"x": 0.0, "y": 0.0, "z": 0.0}

    async def tauff_sender():
        while True:
            msg = MavlinkMessage(
                message_name="DEBUG_VECT", system_id=0, component_id=0,
                target_system_id=0, target_component_id=0,
                fields_json=(
                    '{"name": "TAU_FF", "time_usec": 0, '
                    f'"x": {current_tauff["x"]}, "y": {current_tauff["y"]}, "z": {current_tauff["z"]}}}'
                ),
            )
            try:
                await drone.mavlink_direct.send_message(msg)
            except Exception as exc:
                print(f"  [!!! tauff_sender 예외] {exc}")
            await asyncio.sleep(TAUFF_PERIOD_S)

    tauff_task = asyncio.create_task(tauff_sender())

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"../../logs/pinn_rotation_correction_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "wind_label", "phase", "t_s", "tau_x", "tau_y", "tau_z",
        "tauff_x", "tauff_y", "tauff_z",
        "roll_deg", "pitch_deg", "wx_rad_s", "wy_rad_s",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    async def run_trial(wind_label, phase_name, use_correction):
        corrector.buffer.clear()
        n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
        next_log = time.monotonic()
        peak_att_error = 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]
            wx, wy = latest_gyro["wx"], latest_gyro["wy"]
            vn, ve = latest_pv["vn"], latest_pv["ve"]

            r_cos, r_sin, p_cos, p_sin = yaw_decompose(roll, pitch, yaw)
            feat = {"vn_m_s": vn, "ve_m_s": ve, "roll_cos_yaw": r_cos, "roll_sin_yaw": r_sin,
                    "pitch_cos_yaw": p_cos, "pitch_sin_yaw": p_sin,
                    "wx_rad_s": wx, "wy_rad_s": wy, "wz_rad_s": latest_gyro["wz"]}
            corrector.push_state([feat[f] for f in FEATURES])

            tau_x = tau_y = tau_z = 0.0
            if use_correction and corrector.ready():
                tau_x, tau_y, tau_z = corrector.estimate_tau_dist()
                ff_x = max(-MAX_TAU_FF, min(MAX_TAU_FF, -TAU_FF_GAIN * tau_x))
                ff_y = max(-MAX_TAU_FF, min(MAX_TAU_FF, -TAU_FF_GAIN * tau_y))
                ff_z = max(-MAX_TAU_FF, min(MAX_TAU_FF, -TAU_FF_GAIN * tau_z))
                current_tauff["x"] = ff_x
                current_tauff["y"] = ff_y
                current_tauff["z"] = ff_z
            else:
                current_tauff["x"] = current_tauff["y"] = current_tauff["z"] = 0.0

            att_error = (roll ** 2 + pitch ** 2) ** 0.5
            peak_att_error = max(peak_att_error, att_error)

            writer.writerow([
                wind_label, phase_name, f"{t:.2f}", f"{tau_x:.4f}", f"{tau_y:.4f}", f"{tau_z:.4f}",
                f"{current_tauff['x']:.3f}", f"{current_tauff['y']:.3f}", f"{current_tauff['z']:.3f}",
                f"{roll:.3f}", f"{pitch:.3f}", f"{wx:.4f}", f"{wy:.4f}",
            ])

            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        return peak_att_error

    results = []
    for wind_label, wind_vx, wind_vy in WIND_CONDITIONS:
        print(f"\n{'='*60}\n=== 바람 조건: {wind_label} (vx={wind_vx}, vy={wind_vy}) ===\n{'='*60}")

        print("  -- 회전 보정 OFF --")
        await set_wind(0.0, 0.0)
        await asyncio.sleep(CALM_SETTLE_S)
        await set_wind(wind_vx, wind_vy)
        peak_off = await run_trial(wind_label, "rot_off", use_correction=False)
        print(f"    peak_attitude_error(OFF) = {peak_off:.3f}deg")

        await set_wind(0.0, 0.0)
        current_tauff["x"] = current_tauff["y"] = current_tauff["z"] = 0.0
        await asyncio.sleep(PHASE_GAP_S)

        print("  -- 회전 보정 ON --")
        await asyncio.sleep(CALM_SETTLE_S)
        await set_wind(wind_vx, wind_vy)
        peak_on = await run_trial(wind_label, "rot_on", use_correction=True)
        print(f"    peak_attitude_error(ON)  = {peak_on:.3f}deg")

        improvement = (peak_off - peak_on) / peak_off * 100 if peak_off > 1e-6 else 0.0
        print(f"    개선율 = {improvement:+.1f}%")
        results.append((wind_label, wind_vx, wind_vy, peak_off, peak_on, improvement))

        await set_wind(0.0, 0.0)
        current_tauff["x"] = current_tauff["y"] = current_tauff["z"] = 0.0
        await asyncio.sleep(PHASE_GAP_S)

    csv_file.close()

    print(f"\n{'='*70}\n=== 전체 요약 (자세 피크 편차, deg) ===\n{'='*70}")
    print(f"{'조건':10s} {'풍속(m/s)':>10s} {'OFF peak':>10s} {'ON peak':>10s} {'개선율':>8s}")
    for label, vx, vy, off, on, imp in results:
        speed = (vx ** 2 + vy ** 2) ** 0.5
        print(f"{label:10s} {speed:10.2f} {off:10.3f} {on:10.3f} {imp:+7.1f}%")

    n_improved = sum(1 for r in results if r[5] > 0)
    print(f"\n개선된 조건: {n_improved}/{len(results)}")
    print(f"CSV 저장됨: {csv_path}")

    await set_wind(0.0, 0.0)

    for task in (sender_task, tauff_task, pv_task, att_task, gyro_task):
        task.cancel()
    for task in (sender_task, tauff_task, pv_task, att_task, gyro_task):
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

    print("\nPINN 회전 보정 테스트 완료.")


if __name__ == "__main__":
    asyncio.run(run())
