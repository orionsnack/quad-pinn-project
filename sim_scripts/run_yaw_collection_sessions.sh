#!/bin/bash
# wind_random_sweep.py를 여러 세션으로 나눠서 반복 실행하며, 세션 사이마다 SITL을
# 완전히 재시작하는 오케스트레이션 스크립트.
#
# 왜 필요한가: SITL을 1시간 넘게 안 끄고 돌린 세션에서 위치 추정이 이상해 보이는
# 결과를 한 번 관측한 적 있음(README.md 트러블슈팅 참고) - 다만 "장시간 실행이 진짜
# 원인"이라는 건 재현 검증을 안 해본 가설일 뿐임. 그래도 세션을 짧게 쪼개서 도는 건
# 이 가설과 별개로 그 자체로 이득이 있음: 세션 하나가 죽어도 그때까지 모은 데이터는
# 안전하고(CSV flush + 다음 세션은 계속 진행), 몇 시간짜리 수집을 무인으로 돌릴 때
# 중간 점검 지점이 자연스럽게 생김. 그래서 가설의 확실성과 무관하게 채택함.
#
# 세션마다 yaw 그리드(--n-yaw)의 "칸 수"는 동일하지만, 그리드 시작점(offset)을 세션마다
# 조금씩 밀어서 돎 - 세션 하나의 그리드를 무작정 촘촘하게(N_YAW를 크게) 잡으면 세션
# 하나가 너무 길어지므로, 대신 짧은 세션 여러 개를 서로 다른 offset으로 돌려서
# 합쳤을 때 훨씬 촘촘해지도록 설계함. 예:
# n-yaw=24(15도 간격)를 4세션 x offset(0/3.75/7.5/11.25도)으로 돌리면, 합쳤을 때
# 96개 지점 x 3.75도 간격의 그리드가 됨 - 세션 하나는 짧게 유지하면서 결과적으로는
# 훨씬 촘촘한 커버리지를 얻음. 바람 조건도 세션마다 다른 시드라 중복 없이 다양해짐.
#
# 사용법:
#   ./run_yaw_collection_sessions.sh --sessions 6 --n-yaw 24 --n-per-yaw 10 --trial-duration 8
#   (옵션 생략 시 기본값: sessions=4, n-yaw=24, n-per-yaw=10, trial-duration=8 ->
#    세션당 240조건, 총 960조건, 세션당 약 40~45분 예상, 합산 yaw 해상도 24*4=96지점/3.75도 간격)
#   --trial-duration은 조건 하나당 관측 시간(초, wind_random_sweep.py의 TRIAL_DURATION_S).
#   예) 2도 간격 + 약 13시간을 원하면:
#   ./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15
#
# 실행 전 조건: PX4-Autopilot이 ~/MyProjects/PX4-Autopilot에 빌드되어 있고,
# px4sim conda 환경이 준비되어 있어야 함 (setup_guide.md 참고). 이 스크립트가 SITL을
# 직접 껐다 켰다 하므로, 실행 전에 다른 터미널에서 SITL을 따로 띄워둘 필요 없음
# (오히려 떠 있으면 시작할 때 같이 정리됨).

set -uo pipefail

PX4_DIR="$HOME/MyProjects/PX4-Autopilot"
PROJECT_DIR="$HOME/MyProjects/quad-pinn-project"
SESSIONS=4
N_YAW=24
N_PER_YAW=10
TRIAL_DURATION=8
SEED_BASE=42
BOOT_WAIT_S=10

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sessions) SESSIONS="$2"; shift 2 ;;
        --n-yaw) N_YAW="$2"; shift 2 ;;
        --n-per-yaw) N_PER_YAW="$2"; shift 2 ;;
        --trial-duration) TRIAL_DURATION="$2"; shift 2 ;;
        --seed-base) SEED_BASE="$2"; shift 2 ;;
        *) echo "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

TOTAL_PER_SESSION=$((N_YAW * N_PER_YAW))
COMBINED_YAW_POINTS=$((N_YAW * SESSIONS))
COMBINED_YAW_STEP=$(awk -v n="$COMBINED_YAW_POINTS" 'BEGIN{printf "%.2f", 360.0/n}')
# 대략적인 총 소요시간 추정치 (조건시간 + yaw회전 + SITL재시작/이착륙 오버헤드 세션당 ~2분 가정)
EST_HOURS=$(awk -v ny="$N_YAW" -v npy="$N_PER_YAW" -v td="$TRIAL_DURATION" -v s="$SESSIONS" \
    'BEGIN{cond_s=ny*npy*(td+2); rot_s=ny*4; overhead_s=120; total=(cond_s+rot_s+overhead_s)*s; printf "%.1f", total/3600}')
echo "=== 세션 ${SESSIONS}개, 세션당 yaw ${N_YAW} x 방향당 ${N_PER_YAW} = ${TOTAL_PER_SESSION}조건 (조건당 관측 ${TRIAL_DURATION}초) ==="
echo "=== 총 예상 조건 수: $((TOTAL_PER_SESSION * SESSIONS)) ==="
echo "=== 세션들을 합친 yaw 해상도: ${COMBINED_YAW_POINTS}지점 (${COMBINED_YAW_STEP}도 간격) ==="
echo "=== 예상 총 소요시간: 약 ${EST_HOURS}시간 (대략적인 추정치) ==="

restart_sitl() {
    echo "  [SITL] 기존 프로세스 정리 중..."
    pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "make px4_sitl" 2>/dev/null
    sleep 3

    echo "  [SITL] 재시작 중..."
    (cd "$PX4_DIR" && HEADLESS=1 make px4_sitl gz_x500_windy > /dev/null 2>&1 &)

    local waited=0
    while ! pgrep -f "px4_sitl_default/bin/px4" > /dev/null; do
        sleep 1
        waited=$((waited + 1))
        if [[ $waited -ge 30 ]]; then
            echo "  [SITL] 경고: 30초 내에 px4 프로세스가 안 뜸. 그래도 진행."
            break
        fi
    done
    echo "  [SITL] px4 프로세스 감지됨 (${waited}s). 안정화 대기 ${BOOT_WAIT_S}s..."
    sleep "$BOOT_WAIT_S"
}

FAILED_SESSIONS=0
for i in $(seq 1 "$SESSIONS"); do
    echo ""
    echo "############################################################"
    echo "# 세션 ${i}/${SESSIONS}"
    echo "############################################################"

    restart_sitl

    session_seed=$((SEED_BASE + i * 1000))
    # 세션마다 그리드를 (360/N_YAW/SESSIONS)*세션순번만큼 밀어서, 세션들을 합쳤을 때
    # 촘촘한 그리드가 되도록 함 (파일 상단 주석 참고)
    yaw_offset=$(awk -v n="$N_YAW" -v s="$SESSIONS" -v idx="$((i - 1))" \
        'BEGIN{printf "%.3f", 360.0/n/s*idx}')
    echo "  [수집] 시작 (seed=${session_seed}, yaw-offset=${yaw_offset}도)"
    if (cd "$PROJECT_DIR/sim_scripts" && \
        conda run -n px4sim python wind_random_sweep.py \
            --n-yaw "$N_YAW" --n-per-yaw "$N_PER_YAW" \
            --trial-duration "$TRIAL_DURATION" \
            --yaw-offset "$yaw_offset" --seed "$session_seed"); then
        echo "  [수집] 세션 ${i} 완료"
    else
        echo "  [수집] 세션 ${i} 실패 (exit code $?) - 다음 세션으로 계속 진행"
        FAILED_SESSIONS=$((FAILED_SESSIONS + 1))
    fi
done

echo ""
echo "############################################################"
echo "# 전체 완료: ${SESSIONS}개 세션 중 $((SESSIONS - FAILED_SESSIONS))개 성공, ${FAILED_SESSIONS}개 실패"
echo "# 결과 CSV: ${PROJECT_DIR}/logs/wind_random_*.csv"
echo "############################################################"

echo "  [정리] 마지막 SITL 종료 중..."
pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
