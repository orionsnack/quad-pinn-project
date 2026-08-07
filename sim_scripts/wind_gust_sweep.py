"""
PINN 바람 추정 모델 학습용 데이터 수집 - gust(시간에 따라 변하는 바람) 버전.
wind_random_sweep.py는 한 조건(트라이얼) 내내 바람이 고정이었음. 이 스크립트는
각 에피소드 안에서 바람이 사인파 형태로 계속 변하도록 해서, 모델이 "고정된 바람을
알아맞히기"가 아니라 "지금 이 순간의 바람을 실시간으로 추적"하는 걸 배우게 함.

바람 모델 (에피소드마다 무작위로 다시 뽑음):
  vx(t) = base_vx + amp_vx * sin(2*pi*t/period + phase_vx)
  vy(t) = base_vy + amp_vy * sin(2*pi*t/period + phase_vy)
Gazebo에는 GUST_UPDATE_INTERVAL_S 간격으로만 값을 갱신해서 보내지만(계단식 근사),
CSV에는 로그를 남기는 매 순간(20Hz)의 정확한 연속함수 값을 정답 라벨로 기록함
(드론이 실제로 느끼는 바람은 계단식이라 약간의 근사 오차는 있음 - 큰 문제는 아님).

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
N_EPISODES = 15
RANDOM_SEED = 123               # wind_random_sweep.py(시드 42)와 겹치지 않게 다른 값 사용
BASE_SPEED_RANGE_MPS = (1.0, 8.0)
AMP_FRACTION_RANGE = (0.3, 0.8)  # base speed 대비 진폭 비율
PERIOD_RANGE_S = (4.0, 10.0)     # 한 번 출렁이는 데 걸리는 시간
EPISODE_DURATION_S = 20.0
GUST_UPDATE_INTERVAL_S = 1.0     # Gazebo에 실제로 새 바람값을 보내는 주기.
                                   # gz topic pub이 매번 외부 프로세스를 새로 띄우는
                                   # 방식이라(subprocess spawn당 체감상 100ms+) 너무
                                   # 자주 부르면 전체 소요시간이 확 늘어남 (0.25s일 때
                                   # 15에피소드에 26분 가까이 걸렸음). gust 주기가
                                   # 4~10초라 1초 간격이면 충분히 부드러움.
WIND_SETTLE_S = 1.0
EPISODE_SETTLE_S = 1.0
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
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


def sample_gust_episodes(n, seed):
    rng = random.Random(seed)
    episodes = []
    for _ in range(n):
        base_speed = rng.uniform(*BASE_SPEED_RANGE_MPS)
        base_angle = math.radians(rng.uniform(0.0, 360.0))
        base_vx = base_speed * math.cos(base_angle)
        base_vy = base_speed * math.sin(base_angle)
        amp = base_speed * rng.uniform(*AMP_FRACTION_RANGE)
        episodes.append({
            "base_vx": base_vx, "base_vy": base_vy,
            "amp_vx": amp * rng.uniform(0.7, 1.0),
            "amp_vy": amp * rng.uniform(0.7, 1.0),
            "period": rng.uniform(*PERIOD_RANGE_S),
            "phase_vx": rng.uniform(0.0, 2 * math.pi),
            "phase_vy": rng.uniform(0.0, 2 * math.pi),
        })
    return episodes


def wind_at(episode, t):
    w = 2 * math.pi * t / episode["period"]
    vx = episode["base_vx"] + episode["amp_vx"] * math.sin(w + episode["phase_vx"])
    vy = episode["base_vy"] + episode["amp_vy"] * math.sin(w + episode["phase_vy"])
    return vx, vy


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
    csv_path = f"../logs/wind_gust_{timestamp_str}.csv"
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

    episodes = sample_gust_episodes(N_EPISODES, RANDOM_SEED)
    print(f"\n총 {N_EPISODES}개 gust 에피소드 수집 시작 "
          f"(에피소드당 {EPISODE_DURATION_S:.0f}초, 예상 총 소요 "
          f"{N_EPISODES*(EPISODE_DURATION_S+WIND_SETTLE_S+EPISODE_SETTLE_S)/60:.1f}분)")

    n_steps = int(EPISODE_DURATION_S / LOG_INTERVAL_S)
    updates_per_log = max(1, round(GUST_UPDATE_INTERVAL_S / LOG_INTERVAL_S))

    for ep_idx, episode in enumerate(episodes):
        vx0, vy0 = wind_at(episode, 0.0)
        await set_wind(vx0, vy0)
        await asyncio.sleep(WIND_SETTLE_S)

        next_log = time.monotonic()
        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            true_vx, true_vy = wind_at(episode, t)

            if i % updates_per_log == 0:
                await set_wind(true_vx, true_vy)

            north, east, down = latest_pv["north"], latest_pv["east"], latest_pv["down"]
            vn, ve, vd = latest_pv["vn"], latest_pv["ve"], latest_pv["vd"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]

            writer.writerow([
                ep_idx, f"{true_vx:.3f}", f"{true_vy:.3f}", f"{t:.2f}",
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

        speed_range = (
            min(math.hypot(*wind_at(episode, tt / 10 * EPISODE_DURATION_S)) for tt in range(11)),
            max(math.hypot(*wind_at(episode, tt / 10 * EPISODE_DURATION_S)) for tt in range(11)),
        )
        print(f"  [{ep_idx+1}/{N_EPISODES}] base=({episode['base_vx']:5.2f},{episode['base_vy']:5.2f}) "
              f"period={episode['period']:.1f}s  풍속범위≈{speed_range[0]:.1f}~{speed_range[1]:.1f}m/s  "
              f"roll={roll:5.1f} pitch={pitch:5.1f}")

        await asyncio.sleep(EPISODE_SETTLE_S)

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

    print("\ngust 바람 데이터 수집 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=N_EPISODES)
    parser.add_argument("--speed-min", type=float, default=BASE_SPEED_RANGE_MPS[0])
    parser.add_argument("--speed-max", type=float, default=BASE_SPEED_RANGE_MPS[1])
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    args = parser.parse_args()
    N_EPISODES = args.n
    BASE_SPEED_RANGE_MPS = (args.speed_min, args.speed_max)
    RANDOM_SEED = args.seed

    asyncio.run(run())
