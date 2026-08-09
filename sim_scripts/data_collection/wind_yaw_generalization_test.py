"""
yaw 일반화 검증용 데이터 수집.

지금까지 모든 학습 데이터(wind_random_*.csv, wind_gust_*.csv)는 SITL 스폰 방향이
거의 고정이라 yaw가 75~93도 좁은 범위에만 몰려있었음 (train_wind_estimator.py의
"주의 2" 참고). roll/pitch를 yaw로 회전시켜 tilt_north/tilt_east feature로 바꾸는
수정(body_tilt_to_inertial)을 했지만, 실제로 다른 yaw에서도 통하는지는 검증한 적이
없었음 - 이 스크립트가 그 검증용 데이터를 모음.

wind_random_sweep.py와 거의 동일한 구조지만:
  1) 이륙 후 먼저 TARGET_YAW_DEG(기본 0도 = 북쪽, 학습 범위와 확실히 다른 방향)로
     회전시키고 그 상태를 유지하며 데이터를 모음
  2) 조건 수가 훨씬 적음(기본 8개) - 정식 학습용이 아니라 스팟체크용
CSV는 wind_random_*/wind_gust_* 글롭 패턴에 안 걸리는 wind_yawtest_ 접두사를 씀
(의도치 않게 학습에 섞여 들어가지 않도록).

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import argparse
import asyncio
import csv
import datetime
import math
import random
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
TARGET_YAW_DEG = 0.0     # 학습 데이터 yaw(75~93도)와 확실히 다른 방향
N_CONDITIONS = 8
WIND_SPEED_RANGE_MPS = (0.0, 10.0)
RANDOM_SEED = 999          # 학습 데이터 시드(42/123 등)와 안 겹치게
TRIAL_DURATION_S = 8.0
WIND_SETTLE_S = 1.0
TRIAL_SETTLE_S = 1.0
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
YAW_SETTLE_TIMEOUT_S = 10.0
YAW_TOLERANCE_DEG = 3.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ


async def set_wind(vx, vy, vz=0.0):
    proc = await asyncio.create_subprocess_exec(
        "gz", "topic", "-t", f"/world/{WORLD_NAME}/wind",
        "-m", "gz.msgs.Wind",
        "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
    )
    await proc.wait()


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
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    sender_task = asyncio.create_task(offboard_sender())

    print(f"\n--- 목표 yaw {TARGET_YAW_DEG:.0f}도로 회전 중 (현재 {latest_att['yaw']:.1f}도) ---")
    current_cmd["yaw"] = TARGET_YAW_DEG
    t_yaw_start = time.monotonic()
    while yaw_diff_deg(latest_att["yaw"], TARGET_YAW_DEG) > YAW_TOLERANCE_DEG:
        if time.monotonic() - t_yaw_start > YAW_SETTLE_TIMEOUT_S:
            print(f"  [경고] {YAW_SETTLE_TIMEOUT_S:.0f}초 내에 목표 yaw 미도달 "
                  f"(현재 {latest_att['yaw']:.1f}도) - 그냥 진행")
            break
        await asyncio.sleep(0.1)
    print(f"  -> yaw {latest_att['yaw']:.1f}도로 안정화")

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"../../logs/wind_yawtest_{TARGET_YAW_DEG:.0f}deg_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "condition_idx", "wind_vx_m_s", "wind_vy_m_s", "t_s",
        "cmd_north_m", "cmd_east_m",
        "actual_north_m", "actual_east_m", "actual_down_m",
        "vn_m_s", "ve_m_s", "vd_m_s",
        "roll_deg", "pitch_deg", "yaw_deg",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    wind_conditions = sample_wind_conditions(N_CONDITIONS, RANDOM_SEED)
    print(f"\n총 {N_CONDITIONS}개 무작위 바람 조건 수집 시작 (yaw={TARGET_YAW_DEG:.0f}도 고정, "
          f"조건당 {TRIAL_DURATION_S:.0f}초)")

    n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)

    for cond_idx, (wind_vx, wind_vy) in enumerate(wind_conditions):
        speed = math.hypot(wind_vx, wind_vy)
        await set_wind(wind_vx, wind_vy)
        await asyncio.sleep(WIND_SETTLE_S)

        next_log = time.monotonic()
        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            north, east, down = latest_pv["north"], latest_pv["east"], latest_pv["down"]
            vn, ve, vd = latest_pv["vn"], latest_pv["ve"], latest_pv["vd"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]

            writer.writerow([
                cond_idx, f"{wind_vx:.3f}", f"{wind_vy:.3f}", f"{t:.2f}",
                f"{current_cmd['north']:.3f}", f"{current_cmd['east']:.3f}",
                f"{north:.3f}", f"{east:.3f}", f"{down:.3f}",
                f"{vn:.3f}", f"{ve:.3f}", f"{vd:.3f}",
                f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
            ])

            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        print(f"  [{cond_idx+1}/{N_CONDITIONS}] wind=({wind_vx:5.2f},{wind_vy:5.2f}) "
              f"speed={speed:4.1f}m/s  roll={roll:5.1f} pitch={pitch:5.1f} yaw={yaw:5.1f}")

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

    print(f"\nyaw 일반화 검증 데이터 수집 완료. yaw={TARGET_YAW_DEG:.0f}도")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaw", type=float, default=TARGET_YAW_DEG)
    parser.add_argument("--n", type=int, default=N_CONDITIONS)
    parser.add_argument("--speed-min", type=float, default=WIND_SPEED_RANGE_MPS[0])
    parser.add_argument("--speed-max", type=float, default=WIND_SPEED_RANGE_MPS[1])
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    args = parser.parse_args()
    TARGET_YAW_DEG = args.yaw
    N_CONDITIONS = args.n
    WIND_SPEED_RANGE_MPS = (args.speed_min, args.speed_max)
    RANDOM_SEED = args.seed

    asyncio.run(run())
