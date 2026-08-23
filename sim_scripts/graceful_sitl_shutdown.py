"""
세션형 오케스트레이션 스크립트(`run_yaw_collection_sessions.sh`,
`run_yaw_gust_collection_sessions.sh`, `repeat_sweep.sh`)가 SITL을 재시작하기
직전에 호출하는 보조 스크립트. 기존엔 OS 시그널(`SIGTERM`, 안 되면 `SIGKILL`)만
써왔는데, 이건 PX4 내부의 정상 종료 루틴(파라미터 flush 포함)을 확실히 거치게
보장하지 못해서 `rootfs/parameters.bson`이 가끔 덜 써진 채로 손상되는 원인으로
지목됨 — GPS 락이 영영 안 되는 증상으로 나타났고, 그때마다 "빌드 손상"으로
오진단하고 `make distclean` 완전 재빌드(10~25분)로 대응해온 이력이 있음
(setup_guide.md 트러블슈팅 표, EXPERIMENTS.md 12-16/30/34/36절).

MAVSDK `Action.shutdown()`은 OS 시그널이 아니라 실제 MAVLink 명령
(`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`)이라 PX4가 스스로 정상 종료 절차(파라미터
저장 포함)를 거치고 꺼짐 — SIGTERM보다 훨씬 안전함. 다만 100% 보장은 아니고
연결 자체가 실패할 수도 있으므로(SITL이 아예 안 떠 있는 첫 호출 등), 실패해도
조용히 넘어감 - 호출부(bash)는 항상 기존 pkill -TERM/-9 fallback을 그대로
유지할 것.

사용: python3 graceful_sitl_shutdown.py [--timeout SECONDS(기본 5)]
"""
import argparse
import asyncio

from mavsdk import System


async def _do_shutdown():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    await drone.action.shutdown()


async def main(timeout_s):
    try:
        await asyncio.wait_for(_do_shutdown(), timeout=timeout_s)
        print("[graceful_sitl_shutdown] 정상 종료 명령 전송됨")
    except Exception as exc:
        print(f"[graceful_sitl_shutdown] 스킵(연결 안 되거나 실패, 정상 - "
              f"뒤이어 pkill로 정리됨): {type(exc).__name__}: {exc}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    asyncio.run(main(args.timeout))
