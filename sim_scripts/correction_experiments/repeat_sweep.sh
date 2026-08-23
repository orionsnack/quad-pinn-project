#!/bin/bash
# correction_experiments/의 A/B 스크립트(pinn_wind_correction_sweep.py 등)를 N회
# 반복 실행하며 매회 SITL을 완전히 새로 띄운다.
#
# 왜 필요한가: 지금까지 이 프로젝트의 모든 A/B 개선율(%)은 조건당 "1회 측정"이었음.
# EXPERIMENTS.md 12-14/12-19절에서 같은 코드·같은 파라미터로도 결과가 크게 안
# 재현되는 걸 실제로 여러 번 겪었으면서도, 반복측정으로 평균±표준편차를 내는
# 인프라는 없었음(12-21절 코드 리뷰 4번 항목). 이 스크립트가 그 구멍을 메운다.
#
# 매회 SITL을 재시작하는 이유: 한 프로세스 안에서 조건만 바꿔가며 반복하면
# "SITL 자체의 상태"가 반복 사이에 새지 않는지 알 수 없음 - 독립시행이 되려면
# 매회 완전히 새 SITL 인스턴스에서 돌아야 함. run_yaw_collection_sessions.sh의
# restart_sitl()과 거의 동일한 로직(+ 12-17절에서 배운 parameters.bson 예방적
# 리셋도 포함).
#
# 사용법:
#   ./repeat_sweep.sh --script pinn_wind_correction_sweep.py --repeats 5
#   ./repeat_sweep.sh --script pinn_wind_correction_gust_sweep.py --repeats 5
#   ./repeat_sweep.sh --script pinn_rotation_correction_test.py --repeats 5 --timeout 600
#   # -- 뒤의 인자는 그대로 대상 스크립트에 전달됨(예: settle-time 실험):
#   ./repeat_sweep.sh --script pinn_wind_correction_sweep.py --repeats 5 -- --calm-settle-s 5 --phase-gap-s 4

set -uo pipefail

PX4_DIR="$HOME/MyProjects/PX4-Autopilot"
PROJECT_DIR="$HOME/MyProjects/quad-pinn-project"
PX4SIM_PYTHON="$HOME/miniconda3/envs/px4sim/bin/python"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TARGET_SCRIPT=""
REPEATS=5
BOOT_WAIT_S=15
# gust_sweep.py류(4조건)는 실측 12~13분(gz topic pub 1회당 ~3.5초, 조건당 18회
# 호출) 걸림 - 600초는 부족해서 매번 exit 124로 실패함(EXPERIMENTS.md 12-36절
# 실측). 900초로 올려서 기본값만으로도 안전하게 함 - 더 짧게 걸리는 스크립트
# (예: 회전 테스트 단일조건)는 그냥 더 일찍 끝날 뿐이라 문제없음.
TIMEOUT_S=900
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --script) TARGET_SCRIPT="$2"; shift 2 ;;
        --repeats) REPEATS="$2"; shift 2 ;;
        --timeout) TIMEOUT_S="$2"; shift 2 ;;
        --boot-wait) BOOT_WAIT_S="$2"; shift 2 ;;
        --) shift; EXTRA_ARGS=("$@"); break ;;
        *) echo "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

if [[ -z "$TARGET_SCRIPT" || ! -f "$SCRIPT_DIR/$TARGET_SCRIPT" ]]; then
    echo "사용법: $0 --script <correction_experiments/ 안의 .py 파일명> [--repeats N] [--timeout SEC]"
    exit 1
fi

# 중복 실행 방지 (b604b7a 커밋에서 도입된 패턴과 동일한 이유)
LOCKFILE="/tmp/repeat_sweep.lock"
exec 200>"$LOCKFILE"
if ! flock -n 200; then
    echo "이미 다른 repeat_sweep.sh가 실행 중입니다. 종료."
    exit 1
fi

restart_sitl() {
    echo "  [SITL] MAVSDK로 정상 종료 시도 (parameters.bson 손상 예방, setup_guide.md 참고)..."
    "$PX4SIM_PYTHON" "$PROJECT_DIR/sim_scripts/graceful_sitl_shutdown.py" --timeout 5 2>&1 | sed 's/^/    /'
    sleep 1
    echo "  [SITL] 정리 중..."
    # 주의: pkill -f는 이 스크립트를 실행 중인 셸 자신의 커맨드라인(예: 이 스크립트를
    # 호출한 상위 프로세스의 인자 목록에 스크립트 파일명이 그대로 노출되는 환경)까지
    # 매칭해서 자기 자신을 죽이는 사고가 날 수 있음(실제로 겪음, 2026-08-19) - 반드시
    # "python 실행 경로 + -u + 파일명"처럼 그 프로세스만 유일하게 식별하는 패턴을 쓸 것,
    # 파일명 하나만 단독으로 pkill -f 패턴에 넣지 말 것.
    pkill -TERM -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -TERM -f "gz sim" 2>/dev/null
    pkill -TERM -f "make px4_sitl" 2>/dev/null
    pkill -TERM -f "mavsdk_server" 2>/dev/null
    pkill -TERM -f "bin/python -u $TARGET_SCRIPT" 2>/dev/null

    local waited=0
    while pgrep -f "px4_sitl_default/bin/px4|gz sim|mavsdk_server|bin/python -u $TARGET_SCRIPT" > /dev/null \
          && [[ $waited -lt 8 ]]; do
        sleep 1
        waited=$((waited + 1))
    done

    pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "make px4_sitl" 2>/dev/null
    pkill -9 -f "mavsdk_server" 2>/dev/null
    pkill -9 -f "bin/python -u $TARGET_SCRIPT" 2>/dev/null
    local leftover
    leftover=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|mavsdk_server")
    if [[ -n "$leftover" ]]; then
        echo "$leftover" | xargs -r kill -9
    fi

    rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
    # 12-17절에서 배운 예방적 리셋: 반복 재시작 중 자기장 캘리브레이션 파일이
    # 오염되어 GPS/EKF2가 고착되는 문제를 겪었음 - 비용이 거의 없으므로 매회 리셋.
    rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/parameters.bson"
    sleep 2

    echo "  [SITL] 재시작 중..."
    (cd "$PX4_DIR" && HEADLESS=1 make px4_sitl gz_x500_windy > /dev/null 2>&1 &)

    local w=0
    while ! pgrep -f "px4_sitl_default/bin/px4" > /dev/null; do
        sleep 1
        w=$((w + 1))
        if [[ $w -ge 30 ]]; then
            echo "  [경고] 30초 내에 px4 프로세스가 안 뜸. 그래도 진행."
            break
        fi
    done
    echo "  [SITL] px4 감지됨 (${w}s). 안정화 대기 ${BOOT_WAIT_S}s..."
    sleep "$BOOT_WAIT_S"
}

RUN_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="$PROJECT_DIR/logs/repeat_sweep_${RUN_TIMESTAMP}_${TARGET_SCRIPT%.py}_manifest.txt"
OK_COUNT=0

echo "=== ${TARGET_SCRIPT} 를 ${REPEATS}회 반복 (매회 SITL 새로 시작) ==="

for i in $(seq 1 "$REPEATS"); do
    echo ""
    echo "--- 반복 ${i}/${REPEATS} ---"
    restart_sitl

    before=$(ls -t "$PROJECT_DIR"/logs/*.csv 2>/dev/null | head -1)
    if (cd "$SCRIPT_DIR" && timeout -k 15 "${TIMEOUT_S}s" "$PX4SIM_PYTHON" -u "$TARGET_SCRIPT" "${EXTRA_ARGS[@]}"); then
        after=$(ls -t "$PROJECT_DIR"/logs/*.csv 2>/dev/null | head -1)
        if [[ "$after" != "$before" ]]; then
            echo "$after" >> "$MANIFEST"
            OK_COUNT=$((OK_COUNT + 1))
            echo "  -> 반복 ${i} 성공: $after"
        else
            echo "  [경고] 새 CSV를 못 찾음 (반복 ${i})"
        fi
    else
        echo "  -> 반복 ${i} 실패 (exit $?), 다음으로 계속"
    fi
done

echo ""
echo "=== 완료: ${OK_COUNT}/${REPEATS}회 성공 ==="
echo "매니페스트: $MANIFEST"

pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null

if [[ "$OK_COUNT" -gt 0 ]]; then
    echo ""
    echo "=== 집계 ==="
    "$HOME/miniconda3/bin/python3" "$SCRIPT_DIR/aggregate_repeats.py" "$MANIFEST"
fi
