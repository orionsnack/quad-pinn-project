"""
PINN 바람 추정 모델 학습용 데이터 수집.
wind_sweep_baseline.py(고정 5조건)보다 훨씬 다양한 바람 조건(무작위 세기/방향)을
많이 훑어서, offline_training/의 PINN이 일반화 가능하도록 학습 데이터를 만든다.

yaw(기수 방향)도 그리드로 훑음 (2026-08-08 추가): 원래는 SITL 스폰 방향(거의 고정)
에서만 데이터를 모았는데, 그 상태로 학습한 모델이 다른 yaw에서 거의 못 쓸 정도로
안 통한다는 게 확인됨 (offline_training/train_wind_estimator.py "주의 2" 참고).
N_YAW개 방향으로 고르게 회전해가며, 각 방향에서 N_PER_YAW개의 무작위 바람 조건을
수집 - yaw는 그리드(고른 커버리지가 중요), 바람은 무작위(다양성이 중요)로 설계.
`--yaw-offset`으로 그리드 시작점을 밀 수 있음 - 한 세션의 그리드를 너무 촘촘하게
(N_YAW를 너무 크게) 잡으면 세션 하나가 너무 길어져 SITL 드리프트 위험 구간에
들어가므로, 대신 여러 세션을 서로 다른 offset으로 짧게 나눠 돌려서 합쳤을 때
촘촘해지도록 설계함 (run_yaw_collection_sessions.sh가 세션마다 offset을 자동 계산).

한 번의 이륙-착륙 세션 안에서 전부 처리. 각 yaw 그룹의 (wind_vx, wind_vy)는 우리가
gz topic으로 직접 설정한 값이므로 100% 정확한 정답 라벨로 CSV에 같이 기록됨
(지도학습 가능).

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
import sys
import time
import traceback
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# 백그라운드/리다이렉트로 돌려도 진행상황이 실시간으로 보이게 항상 flush
# (run_yaw_collection_sessions.sh는 -u로 부르지만, 이 파일을 직접 실행할 때도
# 안전하게 동작하도록 wind_gust_sweep.py와 동일하게 맞춤 - 2026-08-11)
print = functools.partial(print, flush=True)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
N_YAW = 24                            # 0~345도, 15도 간격으로 고르게 (--yaw-offset과 조합)
N_PER_YAW = 10                        # yaw 하나당 무작위 바람 조건 수 (총 N_YAW*N_PER_YAW)
YAW_OFFSET_DEG = 0.0                  # 그리드 전체를 이만큼 밀어서 시작 (세션마다 다르게
                                       # 주면, 세션들을 합쳤을 때 그리드가 더 촘촘해짐 -
                                       # run_yaw_collection_sessions.sh가 자동으로 계산해서 줌)
WIND_SPEED_RANGE_MPS = (0.0, 10.0)   # 균등분포 샘플링 범위
RANDOM_SEED = 42                      # 재현 가능하게 고정
TRIAL_DURATION_S = 8.0
WIND_SETTLE_S = 1.0
TRIAL_SETTLE_S = 1.0
SAFE_ALTITUDE_M = 1.5
YAW_SETTLE_TIMEOUT_S = 10.0
# 연결~offboard 시작(_preflight) 전체에 걸리는 시간 상한. SITL 재시작 직후 GPS/텔레메트리
# 스트림이 안 올라오면 이 안의 어느 단계에서든 무한정 멈출 수 있어서 하나로 묶어 방지함.
PREFLIGHT_TIMEOUT_S = 90.0
YAW_TOLERANCE_DEG = 3.0
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


def yaw_diff_deg(a, b):
    d = (a - b + 180) % 360 - 180
    return abs(d)


async def _preflight(drone):
    """연결부터 offboard 진입까지. SITL 재시작 직후 GPS/텔레메트리 스트림이 안 올라오면
    이 단계 어딘가(연결/health/arm/takeoff/텔레메트리 첫 값 대기/offboard 시작)에서
    영원히 멈출 수 있어서, run()에서 이 함수 전체를 asyncio.wait_for로 감싸 하나의
    타임아웃으로 관리함 (단계별로 따로 타임아웃을 붙이면 새 단계가 생길 때마다 또
    빼먹기 쉬움)."""
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

    # 2026-08-11 추가: 회전 토크 물리손실(Jω̇=τ_motor+τ_disturbance)에 필요한 각속도/
    # 모터명령 로깅 (offline_training/wind_pinn_model.py 참고)
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

    # actuator_output_status는 기본 스트림 rate가 0이라 명시적으로 요청해야 값이 옴
    # (attitude_angular_velocity_body 등 다른 스트림과 다른 점 - 처음 이 필드 추가할 때
    # 90초 넘게 멈춰서 실제로 겪은 문제).
    await drone.telemetry.set_rate_actuator_output_status(20.0)

    async def actuator_monitor():
        async for status in drone.telemetry.actuator_output_status():
            latest_actuator["u"] = list(status.actuator[:4])

    actuator_task = asyncio.create_task(actuator_monitor())
    while latest_actuator["u"] is None:
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
    await drone.offboard.start()

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
        raise RuntimeError(
            f"이륙 준비(연결~offboard 시작)가 {PREFLIGHT_TIMEOUT_S:.0f}초를 넘음 "
            "(SITL 재시작 문제 가능성)")
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    # --- CSV 파일 준비 ---
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    yaw_offset_str = f"{YAW_OFFSET_DEG:.1f}".replace(".", "p")
    speed_str = f"{WIND_SPEED_RANGE_MPS[0]:g}-{WIND_SPEED_RANGE_MPS[1]:g}"
    csv_path = (f"../../logs/wind_random_{timestamp_str}"
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
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

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

        # yaw별로 독립적이되 재현 가능하게 시드 오프셋
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

                writer.writerow([
                    cond_idx, f"{wind_vx:.3f}", f"{wind_vy:.3f}", f"{t:.2f}",
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

            print(f"  [{cond_idx+1}/{total_conditions}] yaw={yaw:5.1f} "
                  f"wind=({wind_vx:5.2f},{wind_vy:5.2f}) speed={speed:4.1f}m/s  "
                  f"roll={roll:5.1f} pitch={pitch:5.1f}")

            # 밤새 무인으로 돌 수 있으므로, 중간에 죽어도 여기까지는 안전하게 남도록
            # 조건마다 디스크에 flush (원래는 맨 끝에만 close()해서 다 날아갈 위험 있었음)
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

    await set_wind(5.0, 2.0)  # 기본값 복원

    all_tasks = (sender_task, pv_task, att_task, gyro_task, actuator_task)
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

    print("\n무작위 바람 데이터 수집 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-yaw", type=int, default=N_YAW, help="yaw 그리드 개수 (0~360도를 등분)")
    parser.add_argument("--n-per-yaw", type=int, default=N_PER_YAW, help="yaw 하나당 무작위 바람 조건 수")
    parser.add_argument("--yaw-offset", type=float, default=YAW_OFFSET_DEG,
                         help="yaw 그리드 시작점을 이만큼 밀기 - 여러 세션을 다른 offset으로 "
                              "돌리면 합쳤을 때 더 촘촘한 그리드가 됨")
    parser.add_argument("--speed-min", type=float, default=WIND_SPEED_RANGE_MPS[0])
    parser.add_argument("--speed-max", type=float, default=WIND_SPEED_RANGE_MPS[1])
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    parser.add_argument("--trial-duration", type=float, default=TRIAL_DURATION_S,
                         help="조건 하나당 관측 시간(초)")
    args = parser.parse_args()
    N_YAW = args.n_yaw
    N_PER_YAW = args.n_per_yaw
    YAW_OFFSET_DEG = args.yaw_offset
    WIND_SPEED_RANGE_MPS = (args.speed_min, args.speed_max)
    RANDOM_SEED = args.seed
    TRIAL_DURATION_S = args.trial_duration

    # mavsdk_server(자식 프로세스)/grpc.aio 쪽 정리가 asyncio.run() 종료 시 멈추는 경우가
    # 있어서(트레이스백은 찍히는데 프로세스는 안 죽음), 일반 종료 대신 os._exit()로
    # 인터프리터/스레드 정리 과정을 건너뛰고 바로 죽임 - 오케스트레이션 스크립트의
    # 재시도가 실제로 넘어가려면 이 프로세스가 반드시 빨리 죽어야 함.
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
