"""correction_experiments 안의 4개 스크립트(`pinn_correction_param_tuning.py`,
`pinn_wind_correction_sweep.py`, `pinn_wind_correction_gust_sweep.py`,
`pinn_rotation_correction_test.py`)에 토씨 하나 안 틀리고 복붙돼있던 유틸(바람 설정,
PINN 추정 버퍼, 스무딩, deadband)을 하나로 모음(EXPERIMENTS.md 12-21절 2번,
12-31/12-33절 - 복붙 상태 때문에 `set_wind()` 재발행 버그를 두 파일에서 따로
발견/수정해야 했던 게 이 정리의 직접적인 계기).

이후 EXPERIMENTS.md 12-43절에서 같은 4개 스크립트에 복붙돼있던 텔레메트리/오프보드
"진입부" 보일러플레이트(연결·GPS확인, arm+이륙, pv/att/gyro 모니터, offboard 진입)도
추가로 통합함. 실제 바람/토크를 계속 흘려보내는 offboard_sender 루프 자체는 4개
스크립트마다 페이로드가 서로 달라(가속도 vs 토크 DEBUG_VECT, 더더링 유무, 적응형
게인 유무) 그대로 둠 - 12-33절 때와 같은 판단(중복은 적고 위험은 큰 부분은 안 건드림).

순수 리팩터링 - 로직/동작은 원본과 동일하게 유지함. `arm_and_takeoff`/
`start_telemetry_monitors`/`start_offboard_hold`는 원본 4개 스크립트에서 print
문구가 스크립트마다 미세하게 달랐던 부분(예: 회전 스크립트는 "-> Armed"와
"안전 고도 도달 대기 중..." 줄이 없었음)을 더 자세한 쪽으로 통일함 - 로그 출력
문구만 살짝 달라지고 동작·타이밍·판정 로직은 전부 동일.
"""
import asyncio
import sys
import time
from pathlib import Path

import torch
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw, AccelerationNed

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "offline_training"))
from wind_pinn_model import WindPINN  # noqa: E402


async def set_wind(world_name, vx, vy, vz=0.0):
    """gz topic pub은 1회성 CLI라, gz-transport 구독자 탐색(discovery, 보통
    수백ms 걸림)이 끝나기 전에 프로세스가 끝나면 CLI는 성공(exit 0)을 보고해도
    실제로 아무도 못 받을 수 있음 - 25회 반복측정 중 1회 실제로 발생 확인됨
    (EXPERIMENTS.md 12-23절). 같은 값(멱등)을 짧은 간격으로 3번 재전송해서
    완화 - 한 번이라도 discovery 이후에 도착하면 됨."""
    last_returncode = None
    for attempt in range(3):
        proc = await asyncio.create_subprocess_exec(
            "gz", "topic", "-t", f"/world/{world_name}/wind",
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


class PINNCorrector:
    """WindPINN 체크포인트 로드 + 슬라이딩 윈도우 버퍼 + 추론.
    출력 인덱싱(풍속 north/east인지 tau_x/y/z인지)은 호출부 책임으로 남김 - 4개
    스크립트가 같은 체크포인트의 서로 다른 출력 채널(풍속 2채널 vs 회전토크 3채널)을
    쓰기 때문에, 여기서는 원시 예측값(`predict_raw()`)만 제공함."""

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
    def predict_raw(self):
        x = torch.tensor(self.buffer, dtype=torch.float32).flatten()
        xn = (x - self.X_mean) / self.X_std
        return self.model(xn.unsqueeze(0)).squeeze(0)


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


async def connect_and_wait_ready(drone):
    """PX4 SITL에 연결하고 GPS/홈 위치가 준비될 때까지 대기."""
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


async def arm_and_takeoff(drone, safe_altitude_m, takeoff_timeout_s):
    """arm+이륙 후 안전 고도 도달까지 대기. 타임아웃 내에 도달 못 하면 착륙시키고
    False 반환(호출부는 이 경우 즉시 return할 것) - 도달하면 True."""
    print("\n--- Arming ---")
    await drone.action.arm()
    print("-> Armed")

    print("\n--- Takeoff ---")
    await drone.action.takeoff()

    print(f"  안전 고도({safe_altitude_m}m) 도달 대기 중...")
    t_start = time.monotonic()
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        if alt >= safe_altitude_m:
            print(f"  -> 안전 고도 도달 (relative_altitude={alt:.2f}m)")
            return True
        if time.monotonic() - t_start > takeoff_timeout_s:
            print(f"  [경고] {takeoff_timeout_s:.0f}초 내에 안전 고도 미도달.")
            break
    await drone.action.land()
    return False


async def start_telemetry_monitors(drone):
    """position/velocity, attitude, gyro 텔레메트리를 각각 최신값 dict로 유지하는
    백그라운드 태스크 3개를 만들고, 셋 다 첫 값이 들어올 때까지 기다린 뒤
    (latest_pv, latest_att, latest_gyro, pv_task, att_task, gyro_task) 반환.
    태스크 3개는 호출부가 종료 시 직접 cancel할 책임을 짐(sender_task 등과 함께
    한 번에 정리하는 기존 패턴 유지)."""
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

    return latest_pv, latest_att, latest_gyro, pv_task, att_task, gyro_task


async def start_offboard_hold(drone, nominal):
    """현재 위치(nominal: north/east/down/yaw)를 setpoint로 offboard 모드 진입.
    실패 시 착륙시키고 False 반환(호출부는 이 경우 즉시 return할 것) - 성공하면 True."""
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
        return False
    return True
