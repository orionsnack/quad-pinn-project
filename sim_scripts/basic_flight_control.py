"""
가장 기본적인 드론 제어 데모: 이륙 -> 전진 -> 호버 -> 착륙.

data_collection/correction_experiments 쪽 스크립트들은 바람 조건 스윕, CSV 로깅 등이
얽혀서 복잡한데, 이건 그런 것 없이 MAVSDK Offboard 제어의 최소 골격만 보여주는 용도.

실행 전 조건: WSL에서 PX4 SITL이 돌고 있어야 함
  HEADLESS=1 make px4_sitl gz_x500        (바람 없는 기본 월드)
  HEADLESS=1 make px4_sitl gz_x500_windy  (바람 있는 월드, 이 스크립트에선 안 씀)
"""

import asyncio
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# ============================================================
# 비행 파라미터
# ============================================================
TAKEOFF_ALTITUDE_M = 5.0     # 이륙 목표 고도
FORWARD_DISTANCE_M = 20.0     # 전진 거리 (북쪽 방향, NED 좌표계 기준)
HOLD_AFTER_MOVE_S = 3.0      # 전진 완료 후 제자리 유지 시간
POSITION_TOLERANCE_M = 1.0   # 목표 위치 도달 판정 오차
MOVE_TIMEOUT_S = 30.0        # 전진 이동 제한 시간
TAKEOFF_TIMEOUT_S = 30.0     # 이륙 고도 도달 제한 시간


async def wait_until_connected(drone):
    print("PX4 SITL에 연결 시도 중...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-> 드론에 연결됨")
            break

    print("GPS/홈 위치 확인 중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-> 전역 위치 및 홈 위치 준비 완료")
            break


async def takeoff_and_wait(drone, target_altitude_m, timeout_s):
    print("\n--- Arming ---")
    await drone.action.arm()
    print("-> Armed")

    print("\n--- Takeoff ---")
    await drone.action.set_takeoff_altitude(target_altitude_m)
    await drone.action.takeoff()

    print(f"  목표 고도({target_altitude_m}m) 도달 대기 중...")
    start = time.monotonic()
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        print(f"  현재 고도: {alt:.2f}m", end="\r")
        if alt >= target_altitude_m * 0.95:
            print(f"\n  -> 목표 고도 도달 (altitude={alt:.2f}m)")
            return True
        if time.monotonic() - start >= timeout_s:
            print(f"\n  [경고] {timeout_s:.0f}초 내에 목표 고도 미도달")
            return False
    return False


async def fly_forward(drone, distance_m, tolerance_m, timeout_s):
    """Offboard로 전환해서 북쪽(NED의 north)으로 distance_m만큼 전진."""
    print(f"\n--- Offboard 진입 + 전진 {distance_m}m ---")

    # 현재 위치를 첫 setpoint로 보내야 Offboard 시작이 허용됨
    async for current in drone.telemetry.position_velocity_ned():
        start_north = current.position.north_m
        start_east = current.position.east_m
        start_down = current.position.down_m
        break
    await drone.offboard.set_position_ned(PositionNedYaw(start_north, start_east, start_down, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard 시작 실패: {error._result.result}")
        await drone.action.land()
        return False

    target_north = start_north + distance_m
    target = PositionNedYaw(target_north, start_east, start_down, 0.0)
    await drone.offboard.set_position_ned(target)

    start = time.monotonic()
    async for position in drone.telemetry.position_velocity_ned():
        north = position.position.north_m
        remaining = target_north - north
        print(f"  전진 중... north={north:.2f}m (남은 거리 {remaining:.2f}m)", end="\r")
        if abs(remaining) <= tolerance_m:
            print(f"\n  -> 목표 위치 도달 (north={north:.2f}m)")
            return True
        if time.monotonic() - start >= timeout_s:
            print(f"\n  [경고] {timeout_s:.0f}초 내에 목표 위치 미도달, 계속 진행")
            return True
    return False


async def land_and_wait(drone):
    print("\n--- Offboard 종료 + Land ---")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard 종료 실패: {error._result.result}")

    await drone.action.land()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-> 착륙 완료, disarmed")
            break


async def run():
    drone = System()

    await wait_until_connected(drone)

    reached = await takeoff_and_wait(drone, TAKEOFF_ALTITUDE_M, TAKEOFF_TIMEOUT_S)
    if not reached:
        print(f"[경고] {TAKEOFF_TIMEOUT_S:.0f}초 내에 목표 고도 미도달, 착륙합니다.")
        await drone.action.land()
        return

    await fly_forward(drone, FORWARD_DISTANCE_M, POSITION_TOLERANCE_M, MOVE_TIMEOUT_S)

    print(f"\n--- {HOLD_AFTER_MOVE_S}초간 제자리 유지 ---")
    await asyncio.sleep(HOLD_AFTER_MOVE_S)

    await land_and_wait(drone)


if __name__ == "__main__":
    asyncio.run(run())
