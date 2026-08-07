"""
PINN 바람 추정 모델 학습용 데이터 수집.
wind_sweep_baseline.py(고정 5조건)보다 훨씬 다양한 바람 조건(무작위 세기/방향)을
많이 훑어서, offline_training/의 PINN이 일반화 가능하도록 학습 데이터를 만든다.

한 번의 이륙-착륙 세션 안에서 N_CONDITIONS개의 무작위 바람 벡터를 순서대로
적용하며 호버링 유지. 각 조건의 (wind_vx, wind_vy)는 우리가 gz topic으로 직접
설정한 값이므로 100% 정확한 정답 라벨로 CSV에 같이 기록됨 (지도학습 가능).

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
N_CONDITIONS = 40
WIND_SPEED_RANGE_MPS = (0.0, 10.0)   # 균등분포 샘플링 범위
RANDOM_SEED = 42                      # 재현 가능하게 고정
TRIAL_DURATION_S = 8.0
WIND_SETTLE_S = 1.0
TRIAL_SETTLE_S = 1.0
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.05   # 20Hz 로깅 (송신 주기와 맞춤 - 윈도우 기반 모델 학습용)
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

    print(f"\n[확인] PX4 SITL이 '{WORLD_NAME}' 월드로 실행 중인지 확인할 것.")

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

    # --- Offboard 송신 전담 태스크 (검증된 패턴) ---
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

    # --- CSV 파일 준비 ---
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"../logs/wind_random_{timestamp_str}.csv"
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
    print(f"\n총 {N_CONDITIONS}개 무작위 바람 조건 수집 시작 "
          f"(조건당 {TRIAL_DURATION_S:.0f}초, 예상 총 소요 "
          f"{N_CONDITIONS*(TRIAL_DURATION_S+WIND_SETTLE_S+TRIAL_SETTLE_S)/60:.1f}분)")

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
              f"speed={speed:4.1f}m/s  roll={roll:5.1f} pitch={pitch:5.1f}")

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

    print("\n무작위 바람 데이터 수집 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=N_CONDITIONS)
    parser.add_argument("--speed-min", type=float, default=WIND_SPEED_RANGE_MPS[0])
    parser.add_argument("--speed-max", type=float, default=WIND_SPEED_RANGE_MPS[1])
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    args = parser.parse_args()
    N_CONDITIONS = args.n
    WIND_SPEED_RANGE_MPS = (args.speed_min, args.speed_max)
    RANDOM_SEED = args.seed

    asyncio.run(run())
