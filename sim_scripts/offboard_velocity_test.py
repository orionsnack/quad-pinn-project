"""
PX4 SITL Offboard 모드 실습 - Velocity Setpoint 스트리밍
QGC 없이 완전히 코드로 Offboard 진입 -> 속도 명령 -> 착륙까지 실행

핵심 개념:
- Offboard 진입 전에 setpoint를 최소 1번 먼저 보내야 함 (PX4 요구사항)
- Offboard 진입 후에는 끊임없이(약 0.5초 이내 주기로) setpoint를 계속 보내야 함
  안 그러면 PX4가 자동으로 Offboard를 종료시키고 페일세이프로 전환됨
- 이 스크립트는 Body frame 기준 velocity(전후/좌우/상하 속도 + yaw rate)를 씀

실행 전 조건: WSL에서 PX4 SITL이 이미 pxh> 상태로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500)
"""

import asyncio
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)


# ============================================================
# 회전 테스트 파라미터 - 여기만 바꿔가며 재테스트하면 됨
# ============================================================
YAW_RATE_DEG = 10.0        # 명령할 yaw 각속도 (deg/s), 양수=시계방향(위에서 볼 때)
ROTATION_DURATION_S = 9.0  # 회전 테스트 지속 시간 (초)
LOG_INTERVAL_S = 0.1       # 로그 출력 간격 (초). 촘촘히 보려면 0.1, 대략만 보려면 0.5


async def test_yaw_rotation(drone, latest_yaw, yaw_rate_deg, duration_s, log_interval_s):
    """
    지정한 yaw_rate_deg로 duration_s 동안 제자리 회전시키면서
    log_interval_s 간격으로 명령값 vs 실제 yaw를 비교 출력.
    """
    n_steps = int(duration_s / 0.1)
    log_every = max(1, int(log_interval_s / 0.1))

    print(f"\n--- 제자리 회전 진단 (yaw_rate={yaw_rate_deg} deg/s, {duration_s:.0f}초) ---")
    yaw_start = latest_yaw["value"]
    print(f"  회전 시작 yaw={yaw_start:.2f} deg")

    for i in range(n_steps):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate_deg)
        )
        if i % log_every == 0:
            yaw = latest_yaw["value"]
            delta = (yaw - yaw_start + 540) % 360 - 180
            print(
                f"  t={i*0.1:.1f}s, 명령 yaw_rate={yaw_rate_deg:.1f} deg/s, "
                f"현재 yaw={yaw:.2f} deg, 시작 대비 차이={delta:.1f} deg"
            )
        await asyncio.sleep(0.1)

    yaw_end = latest_yaw["value"]
    print(f"  회전 종료 yaw={yaw_end:.2f} deg")


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
    # (5초 안에 목표 고도(기본 MIS_TAKEOFF_ALT~2.5m)에 못 미친 채로 Offboard에
    #  진입하면, vz=0 명령이 그 낮은 고도를 그대로 고정시켜버림. 지면 근처에서는
    #  yaw rate 명령이 정상적으로 반영되지 않는 것으로 확인됨 - yaw_rate_sweep_test.py
    #  의 .ulg 로그 분석 참고)
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
                  f"(현재 relative_altitude={alt:.2f}m).")
            break
    if not reached_altitude:
        print("  이대로 진행하면 결과가 신뢰할 수 없을 수 있음. Land 후 종료.")
        await drone.action.land()
        return

    # --- Offboard 진입 전, setpoint 최소 1번 선행 전송 (필수) ---
    print("\n--- Offboard 진입 준비: 초기 setpoint 전송 ---")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    print("\n--- Offboard 모드 시작 ---")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        print("-> Land로 안전 착륙 후 종료")
        await drone.action.land()
        return

    # --- yaw 실시간 모니터링용 백그라운드 태스크 ---
    # 필요할 때만 anext() 호출하는 방식은 스트림 메시지가 계속 쌓이기만 해서
    # "밀린" 과거 값을 읽게 되는 문제가 있었음. 대신 별도 태스크가 끊임없이
    # 스트림을 읽으면서 최신값만 변수에 갱신해두고, 본문에서는 그 변수를
    # 그냥 읽기만 하는 방식으로 처리.
    latest_yaw = {"value": None}

    async def yaw_monitor():
        async for attitude in drone.telemetry.attitude_euler():
            latest_yaw["value"] = attitude.yaw_deg

    monitor_task = asyncio.create_task(yaw_monitor())

    # 첫 값 들어올 때까지 잠깐 대기
    while latest_yaw["value"] is None:
        await asyncio.sleep(0.05)

    # --- 전진 3초 (0->1 m/s 부드럽게 램프업 후 유지), yaw 값 같이 출력 ---
    print("\n--- 전진 3초 (0->1 m/s 램프업) ---")
    for i in range(30):  # 0.1초 간격으로 30번 = 3초
        # 처음 1초(10스텝) 동안 0에서 1 m/s까지 서서히 증가, 이후 1 m/s 유지
        target_vx = min(1.0, (i + 1) * 0.1)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(target_vx, 0.0, 0.0, 0.0)
        )
        if i % 5 == 0:  # 0.5초마다 한 번씩 yaw 값 출력
            print(f"  vx={target_vx:.1f} m/s, 현재 yaw={latest_yaw['value']:.2f} deg")
        await asyncio.sleep(0.1)

    # --- 제자리 정지 2초 ---
    print("\n--- 정지 2초 ---")
    for _ in range(20):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # --- 회전 진단 테스트 (파일 상단의 YAW_RATE_DEG 등 파라미터 사용) ---
    await test_yaw_rotation(
        drone, latest_yaw,
        yaw_rate_deg=YAW_RATE_DEG,
        duration_s=ROTATION_DURATION_S,
        log_interval_s=LOG_INTERVAL_S,
    )

    # 모니터링 태스크 정리
    monitor_task.cancel()
    try:
        await monitor_task
    except asyncio.CancelledError:
        pass

    # --- 정지 후 Offboard 종료 ---
    print("\n--- 정지 후 Offboard 종료 ---")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1)

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

    print("\nOffboard 실습 완료.")


if __name__ == "__main__":
    asyncio.run(run())