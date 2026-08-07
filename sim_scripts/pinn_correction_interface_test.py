"""
PINN 보정 인터페이스(배관) 테스트.

아직 실제 PINN 모델이 없으므로, `compute_correction()` 자리를 나중에 진짜 모델
추론 호출로 그대로 갈아끼울 수 있게 비워두고(지금은 스텁), 대신 알려진 테스트
신호를 주입해서 "상태 읽기 -> 보정값 계산 -> setpoint에 더하기 -> 실제 송신"
배관이 끝까지 제대로 연결되어 있는지 검증한다.

한 번의 호버링 세션 안에서:
  - 앞 절반(CORRECTION_OFF_DURATION_S): 보정값 항상 0 (nominal setpoint 그대로 유지)
  - 뒤 절반(CORRECTION_ON_DURATION_S): 고정된 테스트 보정값(TEST_CORRECTION_*)을
    setpoint에 더해서 송신
드론이 실제로 그 보정만큼 위치를 이동해서 따라가면, 배관이 끝까지 살아있다는
증거가 됨 (로그 확인이 아니라 실제 비행 결과로 검증).

compute_correction()의 시그니처(state, elapsed_s) -> {"north","east","down"}만
유지하면서 내부 구현을 PINN 모델 추론으로 바꾸면 바로 실전 투입 가능.

1차(calm)로는 무풍에서 배관 연결 자체만 확인했음. 이번엔 wind_sweep_baseline.py의
"strong" 조건(vx=8, vy=3 m/s)을 그대로 재현해서, 실제 바람이 PID를 흔드는 와중에도
보정 명령이 왜곡 없이 그대로 전달되고 드론이 정확히 따라가는지 검증한다
(PID가 바람과 씨름 중이어도 보정 인터페이스 자체는 독립적으로 살아있어야 함).

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import asyncio
import csv
import datetime
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
WIND_VX_MPS = 8.0   # wind_sweep_baseline.py의 "strong" 조건 재사용
WIND_VY_MPS = 3.0
CORRECTION_OFF_DURATION_S = 15.0
CORRECTION_ON_DURATION_S = 15.0
TEST_CORRECTION_NORTH_M = 1.5   # 배관 검증용 임의 테스트값 (실제 PINN 출력 아님)
TEST_CORRECTION_EAST_M = -1.0
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.1
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ


async def set_wind(vx, vy, vz=0.0):
    """Gazebo world wind 토픽에 새 바람 벡터를 publish (wind_sweep_baseline.py와 동일)."""
    proc = await asyncio.create_subprocess_exec(
        "gz", "topic", "-t", f"/world/{WORLD_NAME}/wind",
        "-m", "gz.msgs.Wind",
        "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
    )
    await proc.wait()


def compute_correction(state: dict, elapsed_s: float) -> dict:
    """
    PINN 보정 함수 자리. 지금은 모델이 없으므로 배관 검증용 테스트 신호만 반환.
    나중에 이 함수 내부만 PINN 추론 호출로 교체하면 됨 (시그니처 유지).

    state: {"north","east","down","vn","ve","vd","roll","pitch","yaw"} (현재 텔레메트리, m/deg)
    elapsed_s: 이 phase 시작 후 경과 시간(초)
    반환: setpoint에 더해질 보정량 {"north": m, "east": m, "down": m}
    """
    if elapsed_s < CORRECTION_OFF_DURATION_S:
        return {"north": 0.0, "east": 0.0, "down": 0.0}
    return {"north": TEST_CORRECTION_NORTH_M, "east": TEST_CORRECTION_EAST_M, "down": 0.0}


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

    print(f"\n--- 바람 설정: vx={WIND_VX_MPS} vy={WIND_VY_MPS} m/s (wind_sweep 'strong' 조건) ---")
    await set_wind(WIND_VX_MPS, WIND_VY_MPS)

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

    # --- Offboard 명령 송신 전담 백그라운드 태스크 (검증된 패턴 재사용) ---
    # sender는 current_cmd를 그대로 보내기만 함 - 보정값 계산 로직과 완전히 분리.
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

    nominal = {"north": latest_pv["north"], "east": latest_pv["east"],
               "down": latest_pv["down"], "yaw": latest_att["yaw"]}
    current_cmd.update(nominal)

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
    csv_path = f"pinn_interface_test_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "t_s", "correction_active",
        "nominal_north_m", "nominal_east_m",
        "corr_north_m", "corr_east_m",
        "cmd_north_m", "cmd_east_m",
        "actual_north_m", "actual_east_m", "actual_down_m",
        "pos_error_m", "roll_deg", "pitch_deg",
        "flight_mode",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    print(f"\n=== 인터페이스 테스트: {CORRECTION_OFF_DURATION_S:.0f}초 보정 OFF -> "
          f"{CORRECTION_ON_DURATION_S:.0f}초 보정 ON "
          f"(테스트값 north+={TEST_CORRECTION_NORTH_M}m, east+={TEST_CORRECTION_EAST_M}m) ===")

    total_duration = CORRECTION_OFF_DURATION_S + CORRECTION_ON_DURATION_S
    n_steps = int(total_duration / LOG_INTERVAL_S)
    phase_t0 = time.monotonic()
    next_log = time.monotonic()

    for i in range(n_steps):
        t = i * LOG_INTERVAL_S
        state = {
            "north": latest_pv["north"], "east": latest_pv["east"], "down": latest_pv["down"],
            "vn": latest_pv["vn"], "ve": latest_pv["ve"], "vd": latest_pv["vd"],
            "roll": latest_att["roll"], "pitch": latest_att["pitch"], "yaw": latest_att["yaw"],
        }

        # --- 인터페이스 핵심: 상태 읽기 -> 보정값 계산 -> setpoint에 더하기 ---
        corr = compute_correction(state, t)
        current_cmd["north"] = nominal["north"] + corr["north"]
        current_cmd["east"] = nominal["east"] + corr["east"]
        current_cmd["down"] = nominal["down"]

        active = corr["north"] != 0.0 or corr["east"] != 0.0
        pos_error = ((state["north"] - current_cmd["north"]) ** 2
                     + (state["east"] - current_cmd["east"]) ** 2) ** 0.5
        writer.writerow([
            f"{t:.1f}", int(active),
            f"{nominal['north']:.3f}", f"{nominal['east']:.3f}",
            f"{corr['north']:.3f}", f"{corr['east']:.3f}",
            f"{current_cmd['north']:.3f}", f"{current_cmd['east']:.3f}",
            f"{state['north']:.3f}", f"{state['east']:.3f}", f"{state['down']:.3f}",
            f"{pos_error:.3f}", f"{state['roll']:.2f}", f"{state['pitch']:.2f}",
        ])
        if i % 20 == 0:
            print(f"  t={t:5.1f}s  보정{'ON ' if active else 'OFF'}  "
                  f"cmd=({current_cmd['north']:.2f},{current_cmd['east']:.2f})  "
                  f"actual=({state['north']:.2f},{state['east']:.2f})  "
                  f"pos_error={pos_error:.2f}m  roll={state['roll']:.1f} pitch={state['pitch']:.1f}")

        next_log += LOG_INTERVAL_S
        sleep_time = next_log - time.monotonic()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            next_log = time.monotonic()

    csv_file.close()
    print(f"\n인터페이스 테스트 완료. CSV 저장됨: {csv_path}")

    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > 1.5 * SEND_PERIOD_S)
        print(
            f"[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  임계치 초과={n_over}/{len(send_gaps)}"
        )

    await set_wind(5.0, 2.0)  # 바람을 windy.sdf 기본값으로 복원

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

    print("\nPINN 보정 인터페이스 테스트 완료.")


if __name__ == "__main__":
    asyncio.run(run())
