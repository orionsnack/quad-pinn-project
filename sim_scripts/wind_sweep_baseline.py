"""
PX4 SITL Offboard - 바람 외란 조건 스윕 + 반복 베이스라인 수집
wind_disturbance_baseline.py의 단일-조건 버전을 확장:
  1) 같은 바람 조건에서 여러 번 반복(HOVER_REPEATS)해서 baseline 분산까지 확보
  2) `gz topic`으로 런타임에 바람 벡터를 바꿔가며 (WIND_CONDITIONS) 여러 세기/방향 스윕

한 번의 이륙-착륙 세션 안에서 모든 조건을 다 도는 방식. 매 조건 전환마다
착륙 후 재이륙하지 않고, Gazebo의 world wind 토픽(/world/<world>/wind)에
새 바람 벡터를 publish해서 즉시 바꿈 (windy-effects-system이 world sdf의
<wind> 기본값 대신 이 토픽으로 들어온 값을 바로 반영함).

windy.sdf 원본 기본값은 (5, 2, 0) m/s. WIND_CONDITIONS에 그 값도 포함해서
기존 단일 조건 실험과 비교 가능하게 함.

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import asyncio
import csv
import datetime
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"

# (label, wind_vx_m_s, wind_vy_m_s) - ENU 월드 프레임 기준. z(수직)는 0으로 고정.
WIND_CONDITIONS = [
    ("calm", 0.0, 0.0),
    ("light", 2.0, 1.0),
    ("default", 5.0, 2.0),      # windy.sdf 원래 기본값
    ("strong", 8.0, 3.0),
    ("crosswind", 0.0, 6.0),    # 세기 비슷하되 방향만 다른 조건
]
HOVER_REPEATS = 3
CRUISE_REPEATS = 1
HOVER_DURATION_S = 15.0
CRUISE_VX_MPS = 2.0
CRUISE_DURATION_S = 12.0
TRIAL_SETTLE_S = 2.0        # 각 반복 사이 정지 시간
WIND_SETTLE_S = 2.0         # 바람 조건 바뀐 직후 안정화 대기 시간
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.1
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ


async def set_wind(vx, vy, vz=0.0):
    """Gazebo world wind 토픽에 새 바람 벡터를 publish (SITL 재시작 불필요)."""
    proc = await asyncio.create_subprocess_exec(
        "gz", "topic", "-t", f"/world/{WORLD_NAME}/wind",
        "-m", "gz.msgs.Wind",
        "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
    )
    await proc.wait()


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

    print(f"\n[확인] PX4 SITL이 '{WORLD_NAME}' 월드로 실행 중인지 확인할 것 "
          f"(HEADLESS=1 make px4_sitl gz_x500_windy).")

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
            print(f"  [경고] {TAKEOFF_TIMEOUT_S:.0f}초 내에 안전 고도 미도달 "
                  f"(현재 relative_altitude={alt:.2f}m).")
            break
    if not reached_altitude:
        print("  이대로 진행하면 결과가 신뢰할 수 없을 수 있음. Land 후 종료.")
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

    latest_mode = {"value": None}

    async def mode_monitor():
        async for mode in drone.telemetry.flight_mode():
            if latest_mode["value"] != str(mode):
                print(f"  [모드 변경 감지!] {latest_mode['value']} -> {mode}")
            latest_mode["value"] = str(mode)

    mode_task = asyncio.create_task(mode_monitor())
    while latest_mode["value"] is None:
        await asyncio.sleep(0.05)
    print(f"현재 비행 모드: {latest_mode['value']}")

    # --- Offboard 명령 송신 전담 백그라운드 태스크 (검증된 패턴 재사용) ---
    current_cmd = {
        "mode": "position",
        "north": 0.0, "east": 0.0, "down": 0.0, "yaw": 0.0,
        "vx": 0.0, "vy": 0.0, "vz": 0.0, "yawspeed": 0.0,
    }
    send_gaps = []

    async def offboard_sender():
        next_tick = time.monotonic()
        last_sent = None
        while True:
            try:
                if current_cmd["mode"] == "position":
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(current_cmd["north"], current_cmd["east"],
                                        current_cmd["down"], current_cmd["yaw"])
                    )
                else:
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(current_cmd["vx"], current_cmd["vy"],
                                               current_cmd["vz"], current_cmd["yawspeed"])
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
    csv_path = f"wind_sweep_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "wind_label", "wind_vx_m_s", "wind_vy_m_s", "repeat_idx",
        "phase", "t_s",
        "cmd_north_m", "cmd_east_m", "cmd_down_m", "cmd_vx_m_s",
        "actual_north_m", "actual_east_m", "actual_down_m",
        "actual_vn_m_s", "actual_ve_m_s", "actual_vd_m_s",
        "pos_error_m", "roll_deg", "pitch_deg", "yaw_deg", "flight_mode",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    async def log_trial(wind_label, wind_vx, wind_vy, repeat_idx, phase_name, duration_s):
        n_steps = int(duration_s / LOG_INTERVAL_S)
        next_log = time.monotonic()
        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            north, east, down = latest_pv["north"], latest_pv["east"], latest_pv["down"]
            vn, ve, vd = latest_pv["vn"], latest_pv["ve"], latest_pv["vd"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]

            if current_cmd["mode"] == "position":
                pos_error = ((north - current_cmd["north"]) ** 2
                             + (east - current_cmd["east"]) ** 2) ** 0.5
                cmd_row = [f"{current_cmd['north']:.2f}", f"{current_cmd['east']:.2f}",
                           f"{current_cmd['down']:.2f}", ""]
            else:
                pos_error = ""
                cmd_row = ["", "", "", f"{current_cmd['vx']:.2f}"]

            writer.writerow([
                wind_label, wind_vx, wind_vy, repeat_idx, phase_name, f"{t:.1f}",
                *cmd_row,
                f"{north:.3f}", f"{east:.3f}", f"{down:.3f}",
                f"{vn:.3f}", f"{ve:.3f}", f"{vd:.3f}",
                f"{pos_error:.3f}" if pos_error != "" else "",
                f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
                latest_mode["value"],
            ])

            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        if phase_name == "hover":
            print(f"  [{wind_label} vx={wind_vx} vy={wind_vy} rep={repeat_idx}] "
                  f"hover 종료 pos_error={pos_error if pos_error != '' else 0:.3f}m  "
                  f"roll={roll:.1f} pitch={pitch:.1f}")
        else:
            print(f"  [{wind_label} vx={wind_vx} vy={wind_vy} rep={repeat_idx}] "
                  f"cruise 종료 vn={vn:.2f} ve={ve:.2f}")

    # --- 조건 스윕 ---
    for wind_label, wind_vx, wind_vy in WIND_CONDITIONS:
        print(f"\n{'='*60}\n=== 바람 조건: {wind_label} (vx={wind_vx}, vy={wind_vy} m/s) ===\n{'='*60}")
        await set_wind(wind_vx, wind_vy)
        await asyncio.sleep(WIND_SETTLE_S)

        for rep in range(HOVER_REPEATS):
            current_cmd["mode"] = "position"
            current_cmd["north"] = latest_pv["north"]
            current_cmd["east"] = latest_pv["east"]
            current_cmd["down"] = latest_pv["down"]
            current_cmd["yaw"] = latest_att["yaw"]
            await log_trial(wind_label, wind_vx, wind_vy, rep, "hover", HOVER_DURATION_S)
            await asyncio.sleep(TRIAL_SETTLE_S)

        for rep in range(CRUISE_REPEATS):
            current_cmd["mode"] = "velocity"
            current_cmd["vx"] = CRUISE_VX_MPS
            current_cmd["vy"] = 0.0
            current_cmd["vz"] = 0.0
            current_cmd["yawspeed"] = 0.0
            await log_trial(wind_label, wind_vx, wind_vy, rep, "cruise", CRUISE_DURATION_S)
            current_cmd["mode"] = "position"
            current_cmd["north"] = latest_pv["north"]
            current_cmd["east"] = latest_pv["east"]
            current_cmd["down"] = latest_pv["down"]
            current_cmd["yaw"] = latest_att["yaw"]
            await asyncio.sleep(TRIAL_SETTLE_S)

    csv_file.close()
    print(f"\n모든 조건 스윕 완료. CSV 저장됨: {csv_path}")

    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > 1.5 * SEND_PERIOD_S)
        print(
            f"\n[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  임계치 초과={n_over}/{len(send_gaps)}"
        )

    # 바람을 원래 기본값으로 복원 (다음 실험에 영향 주지 않도록)
    await set_wind(5.0, 2.0)

    for task in (sender_task, pv_task, att_task, mode_task):
        task.cancel()
    for task in (sender_task, pv_task, att_task, mode_task):
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

    print("\n바람 스윕 베이스라인 수집 완료.")


if __name__ == "__main__":
    asyncio.run(run())
