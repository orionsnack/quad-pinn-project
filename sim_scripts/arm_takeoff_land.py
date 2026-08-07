"""
PX4 SITL Arm -> Takeoff -> Hover -> Land 제어 스크립트
QGC 없이 완전히 코드로 비행 사이클을 실행함

실행 전 조건: WSL에서 PX4 SITL이 이미 pxh> 상태로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500)

주의: QGroundControl을 동시에 켜놨다면, 지도에서 실제로 드론이
움직이는 걸 같이 볼 수 있음 (권장)
"""

import asyncio
from mavsdk import System


async def run():
    drone = System()

    print("PX4 SITL에 연결 시도 중...")
    # udpin:// 로 명시 (경고 제거, 내가 14540 포트를 열고 PX4의 접속을 기다림)
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

    # 이륙 고도 설정 (기본 2.5m 대신 3m로 지정해봄)
    await drone.action.set_takeoff_altitude(3.0)
    print("이륙 고도 3.0m로 설정")

    print("\n--- Arming ---")
    await drone.action.arm()
    print("-> Armed")

    print("\n--- Takeoff ---")
    await drone.action.takeoff()

    # 이륙 중 고도 몇 번 출력하면서 목표 고도 근처까지 대기
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        print(f"  현재 상대고도: {alt:.2f} m")
        if alt > 2.7:  # 목표 3.0m의 90% 정도 도달하면 충분
            print("-> 목표 고도 근접, 이륙 완료로 판단")
            break

    print("\n--- Hover 5초 유지 ---")
    await asyncio.sleep(5)

    print("\n--- Landing ---")
    await drone.action.land()

    # 착륙 완료(디스암)될 때까지 대기
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-> 착륙 완료 및 디스암 확인")
            break

    print("\n비행 사이클 완료.")


if __name__ == "__main__":
    asyncio.run(run())
