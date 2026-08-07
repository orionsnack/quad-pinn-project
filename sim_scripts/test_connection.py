"""
PX4 SITL 연결 테스트 스크립트
- MAVSDK로 SITL에 연결
- 연결 상태 확인
- 위치/자세 텔레메트리 몇 개 받아서 출력

실행 전 조건: WSL에서 PX4 SITL이 이미 pxh> 상태로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500)
"""

import asyncio
from mavsdk import System


async def run():
    drone = System()

    # PX4 SITL은 컴패니언 컴퓨터용으로 14540 포트에 MAVLink를 보냄
    print("PX4 SITL에 연결 시도 중...")
    await drone.connect(system_address="udp://:14540")

    # 연결될 때까지 대기
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-> 드론에 연결됨!")
            break

    # 홈 위치(GPS) 잡힐 때까지 대기
    print("GPS/홈 위치 확인 중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-> 전역 위치 및 홈 위치 준비 완료")
            break

    # 위치 텔레메트리 5개만 받아서 출력
    print("\n위치 텔레메트리 수신 시작 (5개만 출력):")
    count = 0
    async for position in drone.telemetry.position():
        print(
            f"  위도: {position.latitude_deg:.7f}, "
            f"경도: {position.longitude_deg:.7f}, "
            f"상대고도: {position.relative_altitude_m:.2f} m"
        )
        count += 1
        if count >= 5:
            break

    # 자세(attitude) 텔레메트리 5개도 확인
    print("\n자세(오일러각) 텔레메트리 수신 시작 (5개만 출력):")
    count = 0
    async for attitude in drone.telemetry.attitude_euler():
        print(
            f"  roll: {attitude.roll_deg:.2f} deg, "
            f"pitch: {attitude.pitch_deg:.2f} deg, "
            f"yaw: {attitude.yaw_deg:.2f} deg"
        )
        count += 1
        if count >= 5:
            break

    print("\n연결 테스트 완료.")


if __name__ == "__main__":
    asyncio.run(run())
