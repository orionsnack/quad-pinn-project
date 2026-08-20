"""
PINN 바람 추정 모델의 "전환 구간" 특성 조사용 데이터 수집.

지금까지(wind_random_sweep.py, wind_gust_sweep.py) 모든 트라이얼은 한 트라이얼
안에서 성격이 일정했음(계속 고정 아니면 계속 gust). 게다가 WIND_SETTLE_S로
바람 온셋 직후 구간은 로깅에서 아예 제외해왔음 - 즉 "일정 -> 변동" 또는
"변동 -> 일정"으로 바뀌는 전환 순간 자체는 지금까지 학습/평가 데이터 어디에도
없었음. 실제 비행에서는 오히려 이런 전환이 흔함(잔잔하다가 돌풍, 돌풍 지나가고
다시 잔잔) - 이 스크립트는 그 갭을 메우기 위한 조사용(연구용, 학습 파이프라인에
바로 편입하려는 건 아님) 데이터를 모은다.

에피소드 하나 = N_SEGMENTS(기본 5)개 구간(각 SEGMENT_DURATION_S초)을 이어붙인 것,
구간마다 "고정" 또는 "gust"(사인파, wind_gust_sweep.py와 동일한 진폭비/주기 관례)로
번갈아가며 바뀜. 에피소드 절반은 고정으로 시작, 절반은 gust로 시작해서 균형을 맞춤.
CSV에 `regime`(fixed/gust)과 `t_since_transition_s`(그 구간 시작 후 경과시간)를
같이 기록해서, 나중에 "전환 후 t초 지점의 추정 오차" 분석이 바로 가능하게 함.

yaw 그리드는 안 씀(단일 yaw) - 이 조사의 핵심 질문은 "시간축 전환에 모델이 얼마나
민감한가"이지 yaw 일반화가 아니라서, 스코프를 좁혀 수집 시간을 절약함
(EXPERIMENTS.md 참고).

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import argparse
import asyncio
import csv
import datetime
import functools
import math
import os
import random
import subprocess
import sys
import time
import traceback
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

print = functools.partial(print, flush=True)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
N_EPISODES = 40
N_SEGMENTS = 5                    # 에피소드당 구간 수 (전환 4번/에피소드)
SEGMENT_DURATION_S = 8.0          # 구간 하나 길이
EPISODE_DURATION_S = N_SEGMENTS * SEGMENT_DURATION_S
BASE_SPEED_RANGE_MPS = (2.0, 9.0)  # 너무 약하면 fixed/gust 구분 자체가 무의미해짐
AMP_FRACTION = 0.6                 # wind_gust_sweep.py 관례와 동일
GUST_PERIOD_S = 6.0                # wind_gust_sweep.py 관례와 동일
RANDOM_SEED = 777                  # 다른 수집 스크립트(42/123)와 안 겹치게
WIND_SETTLE_S = 1.0
EPISODE_SETTLE_S = 1.0
GUST_UPDATE_INTERVAL_S = 1.0       # wind_gust_sweep.py와 동일한 이유 (gz topic pub 오버헤드)
SAFE_ALTITUDE_M = 1.5
PREFLIGHT_TIMEOUT_S = 90.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ


async def set_wind(vx, vy, vz=0.0):
    """gz topic pub 침묵 실패 완화 - wind_gust_sweep.py와 동일 (EXPERIMENTS.md 12-23절)."""
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
        raise RuntimeError(f"gz topic pub 실패 (returncode={last_returncode})")


def sample_transition_episodes(n, seed):
    rng = random.Random(seed)
    episodes = []
    for i in range(n):
        speed = rng.uniform(*BASE_SPEED_RANGE_MPS)
        angle = math.radians(rng.uniform(0.0, 360.0))
        base_vx = speed * math.cos(angle)
        base_vy = speed * math.sin(angle)
        start_regime = "gust" if i % 2 == 0 else "fixed"  # 절반씩 균형
        segments = []
        regime = start_regime
        for _ in range(N_SEGMENTS):
            segments.append(regime)
            regime = "fixed" if regime == "gust" else "gust"
        episodes.append({
            "base_vx": base_vx, "base_vy": base_vy,
            "amp_vx": speed * AMP_FRACTION, "amp_vy": speed * AMP_FRACTION,
            "period": GUST_PERIOD_S,
            "segments": segments,
        })
    return episodes


def wind_at(episode, t):
    """구간별 독립 위상(phase=0)으로 계산 - fixed 구간은 항상 base값, gust 구간은
    항상 base값에서 시작해 진동. 그 결과 fixed->gust 전환은 값이 안 튀지만(연속),
    gust->fixed 전환은 값이 살짝 튈 수 있음(진동 도중 끊기고 base로 순간복귀) -
    이건 의도적으로 남겨둠(돌풍이 갑자기 잦아드는 상황의 근사로 나쁘지 않음,
    오히려 이 급변 자체가 조사 대상)."""
    seg_idx = min(int(t // SEGMENT_DURATION_S), N_SEGMENTS - 1)
    t_in_seg = t - seg_idx * SEGMENT_DURATION_S
    regime = episode["segments"][seg_idx]
    if regime == "fixed":
        return episode["base_vx"], episode["base_vy"], regime, t_in_seg
    w = 2 * math.pi * t_in_seg / episode["period"]
    vx = episode["base_vx"] + episode["amp_vx"] * math.sin(w)
    vy = episode["base_vy"] + episode["amp_vy"] * math.sin(w)
    return vx, vy, regime, t_in_seg


async def _preflight(drone):
    """wind_gust_sweep.py의 _preflight와 동일 구조 - 연결/GPS/이륙/모니터링 태스크."""
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
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= SAFE_ALTITUDE_M:
            print(f"  -> 안전 고도 도달 (relative_altitude={position.relative_altitude_m:.2f}m)")
            break

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

    print("\n--- Offboard 진입 준비 ---")
    await drone.offboard.set_position_ned(
        PositionNedYaw(current_cmd["north"], current_cmd["east"],
                        current_cmd["down"], current_cmd["yaw"])
    )
    await drone.offboard.start()
    print("--- Offboard 모드 시작 ---")

    sender_task = asyncio.create_task(offboard_sender())
    return (latest_pv, latest_att, latest_gyro, latest_actuator, current_cmd, send_gaps,
            pv_task, att_task, gyro_task, actuator_task, sender_task)


async def run():
    drone = System()

    try:
        (latest_pv, latest_att, latest_gyro, latest_actuator, current_cmd, send_gaps,
         pv_task, att_task, gyro_task, actuator_task, sender_task) = await asyncio.wait_for(
            _preflight(drone), timeout=PREFLIGHT_TIMEOUT_S)
    except asyncio.TimeoutError:
        raise RuntimeError(f"이륙 준비가 {PREFLIGHT_TIMEOUT_S:.0f}초를 넘음")
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"../../logs/wind_transition_{timestamp_str}_n{N_EPISODES}_seed{RANDOM_SEED}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "condition_idx", "wind_vx_m_s", "wind_vy_m_s", "t_s",
        "regime", "t_since_transition_s",
        "cmd_north_m", "cmd_east_m",
        "actual_north_m", "actual_east_m", "actual_down_m",
        "vn_m_s", "ve_m_s", "vd_m_s",
        "roll_deg", "pitch_deg", "yaw_deg",
        "wx_rad_s", "wy_rad_s", "wz_rad_s",
        "actuator0", "actuator1", "actuator2", "actuator3",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    episodes = sample_transition_episodes(N_EPISODES, RANDOM_SEED)
    n_steps = int(EPISODE_DURATION_S / LOG_INTERVAL_S)
    updates_per_log = max(1, round(GUST_UPDATE_INTERVAL_S / LOG_INTERVAL_S))
    print(f"\n에피소드 {N_EPISODES}개 (각 {EPISODE_DURATION_S:.0f}초, 구간 {N_SEGMENTS}개 x "
          f"{SEGMENT_DURATION_S:.0f}초, 전환 {N_SEGMENTS-1}회/에피소드) 수집 시작 - "
          f"예상 소요 {N_EPISODES*(EPISODE_DURATION_S+WIND_SETTLE_S+EPISODE_SETTLE_S)/60:.1f}분")

    for ep_idx, episode in enumerate(episodes):
        vx0, vy0, _, _ = wind_at(episode, 0.0)
        await set_wind(vx0, vy0)
        await asyncio.sleep(WIND_SETTLE_S)

        next_log = time.monotonic()
        last_sent_wind = (round(vx0, 3), round(vy0, 3))
        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            true_vx, true_vy, regime, t_since = wind_at(episode, t)

            if i % updates_per_log == 0:
                wind_key = (round(true_vx, 3), round(true_vy, 3))
                if wind_key != last_sent_wind:
                    await set_wind(true_vx, true_vy)
                    last_sent_wind = wind_key

            north, east, down = latest_pv["north"], latest_pv["east"], latest_pv["down"]
            vn, ve, vd = latest_pv["vn"], latest_pv["ve"], latest_pv["vd"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]
            wx, wy, wz = latest_gyro["wx"], latest_gyro["wy"], latest_gyro["wz"]
            u = latest_actuator["u"]

            writer.writerow([
                ep_idx, f"{true_vx:.3f}", f"{true_vy:.3f}", f"{t:.2f}",
                regime, f"{t_since:.2f}",
                f"{current_cmd['north']:.3f}", f"{current_cmd['east']:.3f}",
                f"{north:.3f}", f"{east:.3f}", f"{down:.3f}",
                f"{vn:.3f}", f"{ve:.3f}", f"{vd:.3f}",
                f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
                f"{wx:.4f}", f"{wy:.4f}", f"{wz:.4f}",
                f"{u[0]:.4f}", f"{u[1]:.4f}", f"{u[2]:.4f}", f"{u[3]:.4f}",
            ])

            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        speed = math.hypot(episode["base_vx"], episode["base_vy"])
        print(f"  [{ep_idx+1}/{N_EPISODES}] base_speed={speed:.1f}m/s "
              f"start={episode['segments'][0]} segments={episode['segments']}")
        csv_file.flush()
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

    await set_wind(5.0, 2.0)

    for task in (sender_task, pv_task, att_task, gyro_task, actuator_task):
        task.cancel()
    for task in (sender_task, pv_task, att_task, gyro_task, actuator_task):
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

    print("\n전환 데이터 수집 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-episodes", type=int, default=N_EPISODES)
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    args = parser.parse_args()
    N_EPISODES = args.n_episodes
    RANDOM_SEED = args.seed

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
