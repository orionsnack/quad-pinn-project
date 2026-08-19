"""
wind_random_sweep.py의 "회전 토크 더더링(dithering)" 버전.

배경(EXPERIMENTS.md 12-16/12-17절): 기존 데이터는 전부 "보정 없이 순수하게 바람에만
반응한" 상태에서 모았음. 그 데이터로 학습한 모델은 물리손실(J*omega_dot=tau_motor+
tau_dist)을 만족시키는 가장 쉬운 방법으로 "자기 입력에 있는 각속도(wx/wy/wz)를 그냥
미분해서 돌려주는" 지름길을 학습해버렸고, 그 추정치를 실시간 토크 피드포워드로 다시
넣었더니 자기 자신의 입력을 오염시키는 양성 피드백으로 폐루프가 발산함(무필터/LPF
둘 다 실패).

이 스크립트는 그 지름길 자체를 데이터로 막는다: 수집 중에 PX4 rate controller에
(모델 추정치와 무관한) 무작위 토크를 계속 흘려보내면서 바람 스윕을 진행. 이러면
관측되는 각속도 변화 중 상당 부분이 "내가 방금 넣은 토크" 때문이 되고, 물리 라벨
(tau_disturbance = J*omega_dot - tau_motor)은 자동으로 정확함(tau_motor를 실제
모터가 낸 결과값(actuator_output_status, 즉 더더링 토크가 이미 반영된 값)으로
계산하기 때문 - 라벨 계산 코드는 안 고쳐도 됨). 따라서 모델이 "자기 입력 미분"
지름길을 쓰면 더더링 구간에서 오차가 커지므로, 진짜 외부 방해와 상관관계가 있는
다른 신호(vn/ve/roll/pitch)를 쓰도록 학습이 유도됨.

주의(중요) - 이건 폐루프가 아니라 개루프: 더더링 신호는 모델 추정치나 실시간
텔레메트리에서 전혀 파생되지 않는 순수 무작위 신호(OU 프로세스)임. 그래서 "보정이
자기 입력에 되먹임되는" 위험이 원천적으로 없음 - gz topic으로 바람을 흔드는 것과
성격이 같은, 외부에서 거는 알려진 교란일 뿐. 발산 위험 없이 안전하게 돌릴 수 있음.

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy), 그리고 firmware가 tau_disturbance
피드포워드 패치(mc_rate_control.cpp/hpp, DEBUG_VECT("TAU_FF") 채널)로 빌드되어
있어야 함.
"""

import argparse
import asyncio
import csv
import datetime
import functools
import math
import os
import random
import sys
import time
import traceback
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from mavsdk.mavlink_direct import MavlinkMessage

print = functools.partial(print, flush=True)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
N_YAW = 24
N_PER_YAW = 10
YAW_OFFSET_DEG = 0.0
WIND_SPEED_RANGE_MPS = (0.0, 10.0)
RANDOM_SEED = 42
TRIAL_DURATION_S = 8.0
WIND_SETTLE_S = 1.0
TRIAL_SETTLE_S = 1.0
SAFE_ALTITUDE_M = 1.5
YAW_SETTLE_TIMEOUT_S = 10.0
PREFLIGHT_TIMEOUT_S = 90.0
YAW_TOLERANCE_DEG = 3.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

# --- 더더링(무작위 토크 주입) 파라미터 ---
DITHER_TAU_S = 0.3       # OU 프로세스 시상수 - 너무 빠르면 백색잡음에 가까워 실제
                          # 회전 동역학(수백ms) 대역을 못 때림, 너무 느리면 병진처럼
                          # 굼떠져서 "빠른 동역학 데이터"라는 목적과 어긋남
DITHER_SIGMA = 0.10       # 정상상태 표준편차(정규화 토크) - 캘리브레이션 테스트에서
                          # 0.05~0.15가 선형·안정적으로 반응하는 걸 확인한 범위 안쪽
DITHER_CLAMP = 0.15       # 안전 클램프
DITHER_RATE_HZ = 20.0
DITHER_PERIOD_S = 1.0 / DITHER_RATE_HZ


class OUDither:
    """Ornstein-Uhlenbeck 프로세스 기반 무작위 토크 - 평균 0으로 되돌아가려는 성질이
    있어 값이 한쪽으로 계속 쌓이지 않으면서도(랜덤워크와 다름), 매 스텝 독립인 백색
    잡음보다는 시간적으로 매끄러워 실제 물리적 토크 교란과 비슷한 특성을 가짐."""

    def __init__(self, rng, tau_s=DITHER_TAU_S, sigma=DITHER_SIGMA, dt=DITHER_PERIOD_S):
        self.rng = rng
        self.alpha = dt / (tau_s + dt)
        self.noise_scale = sigma * math.sqrt(2 * dt / tau_s)
        self.value = 0.0

    def step(self):
        self.value += -self.alpha * self.value + self.noise_scale * self.rng.gauss(0.0, 1.0)
        self.value = max(-DITHER_CLAMP, min(DITHER_CLAMP, self.value))
        return self.value


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


def sample_wind_conditions(n, seed):
    rng = random.Random(seed)
    conditions = []
    for _ in range(n):
        speed = rng.uniform(*WIND_SPEED_RANGE_MPS)
        angle_deg = rng.uniform(0.0, 360.0)
        vx = speed * math.cos(math.radians(angle_deg))
        vy = speed * math.sin(math.radians(angle_deg))
        conditions.append((vx, vy))
    return conditions


def yaw_diff_deg(a, b):
    d = (a - b + 180) % 360 - 180
    return abs(d)


async def _preflight(drone):
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

    print(f"\n[확인] PX4 SITL이 '{WORLD_NAME}' 월드로 실행 중인지 확인할 것.")

    print("\n--- Arming ---")
    await drone.action.arm()
    print("-> Armed")

    print("\n--- Takeoff ---")
    await drone.action.takeoff()

    print(f"  안전 고도({SAFE_ALTITUDE_M}m) 도달 대기 중...")
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        if alt >= SAFE_ALTITUDE_M:
            print(f"  -> 안전 고도 도달 (relative_altitude={alt:.2f}m)")
            break

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

    latest_gyro = {"wx": None, "wy": None, "wz": None}

    async def gyro_monitor():
        async for av in drone.telemetry.attitude_angular_velocity_body():
            latest_gyro["wx"] = av.roll_rad_s
            latest_gyro["wy"] = av.pitch_rad_s
            latest_gyro["wz"] = av.yaw_rad_s

    gyro_task = asyncio.create_task(gyro_monitor())
    while latest_gyro["wx"] is None:
        await asyncio.sleep(0.05)

    latest_actuator = {"u": None}

    await drone.telemetry.set_rate_actuator_output_status(20.0)

    async def actuator_monitor():
        async for status in drone.telemetry.actuator_output_status():
            latest_actuator["u"] = list(status.actuator[:4])

    actuator_task = asyncio.create_task(actuator_monitor())
    while latest_actuator["u"] is None:
        await asyncio.sleep(0.05)

    current_cmd = {"north": 0.0, "east": 0.0, "down": 0.0, "yaw": 0.0}
    send_gaps = []

    async def offboard_sender():
        next_tick = time.monotonic()
        last_sent = None
        while True:
            try:
                await drone.offboard.set_position_ned(
                    PositionNedYaw(current_cmd["north"], current_cmd["east"],
                                    current_cmd["down"], current_cmd["yaw"])
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

    current_cmd["north"] = latest_pv["north"]
    current_cmd["east"] = latest_pv["east"]
    current_cmd["down"] = latest_pv["down"]
    current_cmd["yaw"] = latest_att["yaw"]

    print("\n--- Offboard 진입 준비: 초기 setpoint(현재 위치 고정) 전송 ---")
    await drone.offboard.set_position_ned(
        PositionNedYaw(current_cmd["north"], current_cmd["east"],
                        current_cmd["down"], current_cmd["yaw"])
    )

    print("\n--- Offboard 모드 시작 ---")
    await drone.offboard.start()

    sender_task = asyncio.create_task(offboard_sender())

    # --- 더더링(무작위 토크 주입) 전담 태스크 ---
    dither_rng = random.Random(RANDOM_SEED + 9000)
    dither = {
        "x": OUDither(dither_rng), "y": OUDither(dither_rng), "z": OUDither(dither_rng),
        "value": {"x": 0.0, "y": 0.0, "z": 0.0},
    }

    async def dither_sender():
        while True:
            dither["value"]["x"] = dither["x"].step()
            dither["value"]["y"] = dither["y"].step()
            dither["value"]["z"] = dither["z"].step()
            msg = MavlinkMessage(
                message_name="DEBUG_VECT", system_id=0, component_id=0,
                target_system_id=0, target_component_id=0,
                fields_json=(
                    '{"name": "TAU_FF", "time_usec": 0, '
                    f'"x": {dither["value"]["x"]:.4f}, '
                    f'"y": {dither["value"]["y"]:.4f}, '
                    f'"z": {dither["value"]["z"]:.4f}}}'
                ),
            )
            try:
                await drone.mavlink_direct.send_message(msg)
            except Exception as exc:
                print(f"  [!!! dither_sender 예외] {type(exc).__name__}: {exc}")
            await asyncio.sleep(DITHER_PERIOD_S)

    dither_task = asyncio.create_task(dither_sender())

    return (latest_pv, latest_att, latest_gyro, latest_actuator, current_cmd, send_gaps,
            dither, pv_task, att_task, gyro_task, actuator_task, sender_task, dither_task)


async def run():
    drone = System()

    try:
        (latest_pv, latest_att, latest_gyro, latest_actuator, current_cmd, send_gaps,
         dither, pv_task, att_task, gyro_task, actuator_task, sender_task,
         dither_task) = await asyncio.wait_for(_preflight(drone), timeout=PREFLIGHT_TIMEOUT_S)
    except asyncio.TimeoutError:
        raise RuntimeError(
            f"이륙 준비(연결~offboard 시작)가 {PREFLIGHT_TIMEOUT_S:.0f}초를 넘음 "
            "(SITL 재시작 문제 가능성)")
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    yaw_offset_str = f"{YAW_OFFSET_DEG:.1f}".replace(".", "p")
    speed_str = f"{WIND_SPEED_RANGE_MPS[0]:g}-{WIND_SPEED_RANGE_MPS[1]:g}"
    csv_path = (f"../../logs/wind_random_dither_{timestamp_str}"
                f"_yaw{yaw_offset_str}_n{N_YAW}x{N_PER_YAW}_speed{speed_str}_seed{RANDOM_SEED}.csv")
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "condition_idx", "wind_vx_m_s", "wind_vy_m_s", "t_s",
        "cmd_north_m", "cmd_east_m",
        "actual_north_m", "actual_east_m", "actual_down_m",
        "vn_m_s", "ve_m_s", "vd_m_s",
        "roll_deg", "pitch_deg", "yaw_deg",
        "wx_rad_s", "wy_rad_s", "wz_rad_s",
        "actuator0", "actuator1", "actuator2", "actuator3",
        "dither_x", "dither_y", "dither_z",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")
    print(f"더더링: OU 프로세스 tau={DITHER_TAU_S}s sigma={DITHER_SIGMA} clamp=±{DITHER_CLAMP}")

    yaw_values = [(i * (360.0 / N_YAW) + YAW_OFFSET_DEG) % 360.0 for i in range(N_YAW)]
    n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
    total_conditions = N_YAW * N_PER_YAW
    print(f"\nyaw {N_YAW}방향(offset={YAW_OFFSET_DEG:.1f}도) x 방향당 바람조건 {N_PER_YAW}개 "
          f"= 총 {total_conditions}개 수집 시작 "
          f"(조건당 {TRIAL_DURATION_S:.0f}초, 예상 총 소요 "
          f"{total_conditions*(TRIAL_DURATION_S+WIND_SETTLE_S+TRIAL_SETTLE_S)/60:.1f}분 "
          f"+ yaw 회전 시간)")

    cond_idx = 0
    for yaw_idx, target_yaw in enumerate(yaw_values):
        print(f"\n{'='*60}\n=== yaw {yaw_idx+1}/{N_YAW}: 목표 {target_yaw:.0f}도로 회전 중 "
              f"(현재 {latest_att['yaw']:.1f}도) ===\n{'='*60}")
        current_cmd["yaw"] = target_yaw
        t_yaw_start = time.monotonic()
        while yaw_diff_deg(latest_att["yaw"], target_yaw) > YAW_TOLERANCE_DEG:
            if time.monotonic() - t_yaw_start > YAW_SETTLE_TIMEOUT_S:
                print(f"  [경고] {YAW_SETTLE_TIMEOUT_S:.0f}초 내에 목표 yaw 미도달 "
                      f"(현재 {latest_att['yaw']:.1f}도) - 그냥 진행")
                break
            await asyncio.sleep(0.1)
        print(f"  -> yaw {latest_att['yaw']:.1f}도로 안정화")

        wind_conditions = sample_wind_conditions(N_PER_YAW, RANDOM_SEED + yaw_idx)

        for wind_vx, wind_vy in wind_conditions:
            speed = math.hypot(wind_vx, wind_vy)
            await set_wind(wind_vx, wind_vy)
            await asyncio.sleep(WIND_SETTLE_S)

            next_log = time.monotonic()
            for i in range(n_steps):
                t = i * LOG_INTERVAL_S
                north, east, down = latest_pv["north"], latest_pv["east"], latest_pv["down"]
                vn, ve, vd = latest_pv["vn"], latest_pv["ve"], latest_pv["vd"]
                roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]
                wx, wy, wz = latest_gyro["wx"], latest_gyro["wy"], latest_gyro["wz"]
                u = latest_actuator["u"]
                dx, dy, dz = dither["value"]["x"], dither["value"]["y"], dither["value"]["z"]

                writer.writerow([
                    cond_idx, f"{wind_vx:.3f}", f"{wind_vy:.3f}", f"{t:.2f}",
                    f"{current_cmd['north']:.3f}", f"{current_cmd['east']:.3f}",
                    f"{north:.3f}", f"{east:.3f}", f"{down:.3f}",
                    f"{vn:.3f}", f"{ve:.3f}", f"{vd:.3f}",
                    f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
                    f"{wx:.4f}", f"{wy:.4f}", f"{wz:.4f}",
                    f"{u[0]:.4f}", f"{u[1]:.4f}", f"{u[2]:.4f}", f"{u[3]:.4f}",
                    f"{dx:.4f}", f"{dy:.4f}", f"{dz:.4f}",
                ])

                next_log += LOG_INTERVAL_S
                sleep_time = next_log - time.monotonic()
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                else:
                    next_log = time.monotonic()

            print(f"  [{cond_idx+1}/{total_conditions}] yaw={yaw:5.1f} "
                  f"wind=({wind_vx:5.2f},{wind_vy:5.2f}) speed={speed:4.1f}m/s  "
                  f"roll={roll:5.1f} pitch={pitch:5.1f}  dither=({dx:+.2f},{dy:+.2f},{dz:+.2f})")

            csv_file.flush()

            cond_idx += 1
            await asyncio.sleep(TRIAL_SETTLE_S)

    csv_file.close()
    print(f"\n데이터 수집 완료. CSV 저장됨: {csv_path}")

    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > 1.5 * SEND_PERIOD_S)
        print(
            f"[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  임계치 초과={n_over}/{len(send_gaps)}"
        )

    await set_wind(5.0, 2.0)

    all_tasks = (sender_task, dither_task, pv_task, att_task, gyro_task, actuator_task)
    for task in all_tasks:
        task.cancel()
    for task in all_tasks:
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

    print("\n무작위 바람+토크 더더링 데이터 수집 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-yaw", type=int, default=N_YAW)
    parser.add_argument("--n-per-yaw", type=int, default=N_PER_YAW)
    parser.add_argument("--yaw-offset", type=float, default=YAW_OFFSET_DEG)
    parser.add_argument("--speed-min", type=float, default=WIND_SPEED_RANGE_MPS[0])
    parser.add_argument("--speed-max", type=float, default=WIND_SPEED_RANGE_MPS[1])
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    parser.add_argument("--trial-duration", type=float, default=TRIAL_DURATION_S)
    args = parser.parse_args()
    N_YAW = args.n_yaw
    N_PER_YAW = args.n_per_yaw
    YAW_OFFSET_DEG = args.yaw_offset
    WIND_SPEED_RANGE_MPS = (args.speed_min, args.speed_max)
    RANDOM_SEED = args.seed
    TRIAL_DURATION_S = args.trial_duration

    try:
        asyncio.run(run())
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
