"""
PX4 SITL Offboard - Yaw Rate 스윕 테스트
여러 개의 (yaw_rate, duration) 조합을 한 비행 세션 안에서 순서대로 테스트하고
결과를 CSV로 기록. 나중에 엑셀/판다스에서 경향성 비교 분석용.

각 테스트 사이에는 짧은 정지 구간을 둬서 이전 회전의 관성이 다음 테스트에
영향을 주지 않도록 함.

실행 전 조건: WSL에서 PX4 SITL이 이미 pxh> 상태로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500)
"""

import asyncio
import csv
import datetime
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)


# ============================================================
# 테스트 매트릭스 - 원하는 조합을 자유롭게 추가/수정
# ============================================================
TEST_CASES = [
    {"yaw_rate_deg": 10.0, "duration_s": 6.0},
    {"yaw_rate_deg": 20.0, "duration_s": 6.0},
    {"yaw_rate_deg": 45.0, "duration_s": 6.0},
    {"yaw_rate_deg": 60.0, "duration_s": 6.0},
    {"yaw_rate_deg": 90.0, "duration_s": 6.0},
]
SETTLE_TIME_S = 2.0   # 각 테스트 사이 정지(관성 제거) 시간
LOG_INTERVAL_S = 0.1  # 로그 기록 간격
SEND_RATE_HZ = 20.0   # offboard 명령 송신 주기 (기존 10Hz -> 20Hz로 여유 확보)
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ
GAP_WARN_THRESHOLD_S = 1.5 * SEND_PERIOD_S  # 이보다 벌어진 송신 간격은 이상으로 기록


def wrap_delta(yaw_now, yaw_ref):
    """yaw_now - yaw_ref 를 -180~180 범위로 정규화."""
    return (yaw_now - yaw_ref + 540) % 360 - 180


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

    # 고정 5초 sleep 대신, 실제 고도가 안전 고도에 도달할 때까지 대기.
    # (기존 방식은 5초 안에 목표 고도(기본 MIS_TAKEOFF_ALT~2.5m)에 못 미쳤을 경우
    #  Offboard의 vz=0 명령이 그 낮은 고도를 그대로 고정(hold)시켜버리는 문제가 있었음.
    #  지면 근처에서는 yaw rate 명령이 정상적으로 반영되지 않는 것으로 확인됨.)
    SAFE_ALTITUDE_M = 1.5
    TAKEOFF_TIMEOUT_S = 15.0
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
                  f"(현재 relative_altitude={alt:.2f}m). 낮은 고도에서 yaw 테스트가 "
                  f"왜곡될 수 있음.")
            break
    if not reached_altitude:
        print("  이대로 진행하면 결과가 신뢰할 수 없을 수 있음. Land 후 종료.")
        await drone.action.land()
        return

    print("\n--- Offboard 진입 준비: 초기 setpoint 전송 ---")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("\n--- Offboard 모드 시작 ---")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return

    # --- Offboard 명령 송신 전담 백그라운드 태스크 ---
    # 기존 방식(메인 루프에서 set_velocity_body 호출 -> 로그/CSV 기록 -> sleep(0.1))은
    # set_velocity_body 자체의 gRPC 왕복 시간과 로깅 작업 시간이 sleep 시간에 그대로
    # 더해져서 실제 송신 간격이 들쭉날쭉해지는 문제가 있었음 (특히 WSL2 미러 네트워킹
    # 모드에서 루프백 트래픽 지연이 튀는 경우가 보고된 바 있음). 이 간격이 PX4 offboard
    # setpoint 신선도 임계치를 넘으면 FlightTaskOffboard가 내부 yaw 적분 기준을 재설정
    # 하면서 "정지 -> 급격한 질주 -> 역방향 되감김" 같은 비결정적 패턴이 나타나는 것으로
    # 추정됨. 송신을 로깅과 완전히 분리하고 드리프트 보정 고정 주기로 돌려서 이를 검증/완화.
    current_cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yawspeed": 0.0}
    send_gaps = []

    async def offboard_sender():
        next_tick = time.monotonic()
        last_sent = None
        while True:
            try:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        current_cmd["vx"], current_cmd["vy"],
                        current_cmd["vz"], current_cmd["yawspeed"],
                    )
                )
            except Exception as exc:
                # 태스크가 조용히 죽어서 이후 명령이 전혀 안 나가는 일을 막기 위해
                # 반드시 표면화하고 다음 주기에 재시도
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
                # 이미 늦었으면 기준 시각을 현재로 재설정해 드리프트 누적 방지
                next_tick = time.monotonic()

    sender_task = asyncio.create_task(offboard_sender())

    # --- yaw 실시간 모니터링용 백그라운드 태스크 ---
    latest_yaw = {"value": None}

    async def yaw_monitor():
        async for attitude in drone.telemetry.attitude_euler():
            latest_yaw["value"] = attitude.yaw_deg

    monitor_task = asyncio.create_task(yaw_monitor())
    while latest_yaw["value"] is None:
        await asyncio.sleep(0.05)

    # --- 비행 모드 실시간 모니터링 태스크 (Offboard 이탈 여부 확인용) ---
    latest_mode = {"value": None, "changed_log": []}

    async def mode_monitor():
        async for mode in drone.telemetry.flight_mode():
            if latest_mode["value"] != str(mode):
                print(f"  [모드 변경 감지!] {latest_mode['value']} -> {mode}")
                latest_mode["changed_log"].append(str(mode))
            latest_mode["value"] = str(mode)

    mode_task = asyncio.create_task(mode_monitor())
    while latest_mode["value"] is None:
        await asyncio.sleep(0.05)
    print(f"현재 비행 모드: {latest_mode['value']}")

    # --- CSV 파일 준비 ---
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"yaw_sweep_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "phase_index", "commanded_yaw_rate_deg_s", "t_s",
        "yaw_deg", "delta_from_phase_start_deg", "empirical_rate_deg_s", "flight_mode"
    ])

    print(f"\nCSV 로그 저장 경로: {csv_path}")

    # --- 테스트 매트릭스 순회 ---
    for phase_idx, case in enumerate(TEST_CASES):
        yaw_rate = case["yaw_rate_deg"]
        duration = case["duration_s"]
        n_steps = int(duration / LOG_INTERVAL_S)

        print(f"\n=== Phase {phase_idx}: yaw_rate={yaw_rate} deg/s, {duration:.0f}초 ===")
        current_cmd["yawspeed"] = yaw_rate  # 송신 태스크가 다음 주기부터 알아서 반영

        phase_start_yaw = latest_yaw["value"]
        prev_yaw = phase_start_yaw
        prev_t = 0.0
        next_log = time.monotonic()
        # wrap_delta(yaw, phase_start_yaw)는 -180~180 범위로만 계산되므로
        # 한 바퀴(360deg) 이상 도는 phase에서는 누적 회전량이 잘못 표시됨.
        # 매 스텝의 step_delta를 계속 더해서(unwrap) 실제 누적 회전량을 추적.
        cumulative_rotation = 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            yaw = latest_yaw["value"]

            # 순간 각속도 추정 (직전 로그 시점 대비)
            step_delta = wrap_delta(yaw, prev_yaw)
            cumulative_rotation += step_delta
            dt = t - prev_t if t > prev_t else LOG_INTERVAL_S
            empirical_rate = step_delta / dt if dt > 0 else 0.0

            writer.writerow([phase_idx, yaw_rate, f"{t:.1f}", f"{yaw:.2f}",
                              f"{cumulative_rotation:.1f}", f"{empirical_rate:.1f}", latest_mode["value"]])
            print(f"  t={t:.1f}s  yaw={yaw:7.2f}  누적회전={cumulative_rotation:7.1f}  "
                  f"실측순간속도≈{empirical_rate:7.1f} deg/s  mode={latest_mode['value']}")
            prev_yaw = yaw
            prev_t = t

            # 로깅 작업 시간과 무관하게 LOG_INTERVAL_S 주기를 유지 (드리프트 보정)
            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        total_delta = cumulative_rotation
        avg_rate = total_delta / duration
        print(f"  -> Phase {phase_idx} 요약: 총 회전량={total_delta:.1f}deg, "
              f"평균 각속도={avg_rate:.1f} deg/s (명령값={yaw_rate} deg/s)")

        # 다음 테스트 전 정지 (관성 제거)
        print(f"  -- 정지 {SETTLE_TIME_S:.0f}초 (다음 테스트 준비) --")
        current_cmd["yawspeed"] = 0.0
        await asyncio.sleep(SETTLE_TIME_S)

    csv_file.close()
    print(f"\n모든 테스트 완료. CSV 저장됨: {csv_path}")

    # --- 실제 offboard 송신 간격 통계 (타이밍 지터 가설 검증용) ---
    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > GAP_WARN_THRESHOLD_S)
        print(
            f"\n[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  "
            f"{GAP_WARN_THRESHOLD_S*1000:.0f}ms 초과 횟수={n_over}/{len(send_gaps)}"
        )
        if n_over > 0:
            print(
                "  -> 송신 간격이 튀는 구간이 있었음. yaw 회전 이상 현상과 시간대가 "
                "겹치는지 CSV의 t_s와 대조해볼 것 (타이밍 지터가 원인일 가능성 뒷받침)."
            )

    # 모니터링/송신 태스크 정리
    sender_task.cancel()
    monitor_task.cancel()
    mode_task.cancel()
    try:
        await sender_task
    except asyncio.CancelledError:
        pass
    try:
        await monitor_task
    except asyncio.CancelledError:
        pass
    try:
        await mode_task
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

    print("\n스윕 테스트 완료.")


if __name__ == "__main__":
    asyncio.run(run())