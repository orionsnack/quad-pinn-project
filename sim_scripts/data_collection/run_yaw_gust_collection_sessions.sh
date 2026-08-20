#!/bin/bash
# wind_gust_sweep.py를 여러 세션으로 나눠서 반복 실행하며, 세션 사이마다 SITL을
# 완전히 재시작하는 오케스트레이션 스크립트. run_yaw_collection_sessions.sh(고정바람용)의
# gust 버전 - 구조/근거는 그 스크립트 상단 주석과 동일함(짧은 세션으로 쪼개는 이유,
# yaw offset으로 그리드 합치는 방식 등).
#
# 목적: 12-30절에서 고정바람은 비더더링 480조건 재수집으로 7.5배 개선됐지만 gust는
# 그 데이터에 없어서 1.7배 개선(0.909m/s)에 그침 - gust도 같은 규모로 비더더링
# 재수집해서 마저 개선하기 위함.
#
# plot_session_grid.py/plot_combined_summary.py는 wind_random_sweep.py 스키마 기준으로
# 짜여 있어 gust CSV(다른 컬럼 구성)와 호환 안 될 수 있음 - 이 스크립트는 그래프 생성을
# 아예 안 함(리스크 줄이기용, 필요하면 나중에 CSV로 수동 분석).
#
# 사용법:
#   ./run_yaw_gust_collection_sessions.sh --sessions 4 --n-yaw 12 --n-per-yaw 10 --seed-base 8042
#   (옵션 생략 시 기본값: sessions=4, n-yaw=12, n-per-yaw=10 ->
#    세션당 120조건, 총 480조건, 세션당 약 45분 예상)
#
# 실행 전 조건: PX4-Autopilot이 ~/MyProjects/PX4-Autopilot에 빌드되어 있고,
# px4sim conda 환경이 준비되어 있어야 함. 이 스크립트가 SITL을 직접 껐다 켰다 하므로,
# 실행 전에 다른 터미널에서 SITL을 따로 띄워둘 필요 없음.

set -uo pipefail

LOCKFILE="/tmp/run_yaw_gust_collection_sessions.lock"
exec 200>"$LOCKFILE"
if ! flock -n 200; then
    echo "[중복 실행 감지] 이미 다른 run_yaw_gust_collection_sessions.sh 인스턴스가 실행 중입니다."
    echo "  (락 파일: $LOCKFILE) - 중복 SITL 충돌 방지를 위해 이 인스턴스는 즉시 종료합니다."
    exit 1
fi

PX4_DIR="$HOME/MyProjects/PX4-Autopilot"
PROJECT_DIR="$HOME/MyProjects/quad-pinn-project"
PX4SIM_PYTHON="$HOME/miniconda3/envs/px4sim/bin/python"
SESSIONS=4
N_YAW=12
N_PER_YAW=10
SEED_BASE=8042
BOOT_WAIT_S=15
START_SESSION=1
MAX_RETRIES=3
EPISODE_DURATION_S=20.0
PER_EPISODE_OVERHEAD_S=2.0    # gz topic pub spawn당 오버헤드 등 - 관측된 실제 페이스가
                               # 예상보다 훨씬 느릴 때(2026-08-21 새벽, ~90s/조건 관측 -
                               # 원래 가정 22s의 4배) --per-episode-overhead-s로 올려서
                               # SESSION_TIMEOUT_S를 현실적으로 재계산할 것. 이 값을 안
                               # 올리면 세션이 실제 완료 전에 항상 강제종료→처음부터
                               # 재시도를 반복하며 데이터를 통째로 날림(세션 크기를
                               # 줄여도 타임아웃도 같이 줄어들어서 비율이 안 바뀜).
SESSION_TIMEOUT_OVERRIDE_S=""  # 직접 초 단위로 지정하고 싶으면 --session-timeout-s

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sessions) SESSIONS="$2"; shift 2 ;;
        --n-yaw) N_YAW="$2"; shift 2 ;;
        --n-per-yaw) N_PER_YAW="$2"; shift 2 ;;
        --seed-base) SEED_BASE="$2"; shift 2 ;;
        --start-session) START_SESSION="$2"; shift 2 ;;
        --episode-duration) EPISODE_DURATION_S="$2"; shift 2 ;;
        --per-episode-overhead-s) PER_EPISODE_OVERHEAD_S="$2"; shift 2 ;;
        --session-timeout-s) SESSION_TIMEOUT_OVERRIDE_S="$2"; shift 2 ;;
        *) echo "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

TOTAL_PER_SESSION=$((N_YAW * N_PER_YAW))
COMBINED_YAW_POINTS=$((N_YAW * SESSIONS))
COMBINED_YAW_STEP=$(awk -v n="$COMBINED_YAW_POINTS" 'BEGIN{printf "%.2f", 360.0/n}')
EST_HOURS=$(awk -v ny="$N_YAW" -v npy="$N_PER_YAW" -v ed="$EPISODE_DURATION_S" -v ov="$PER_EPISODE_OVERHEAD_S" -v s="$SESSIONS" \
    'BEGIN{cond_s=ny*npy*(ed+ov); rot_s=ny*4; overhead_s=120; total=(cond_s+rot_s+overhead_s)*s; printf "%.1f", total/3600}')
if [[ -n "$SESSION_TIMEOUT_OVERRIDE_S" ]]; then
    SESSION_TIMEOUT_S="$SESSION_TIMEOUT_OVERRIDE_S"
else
    SESSION_TIMEOUT_S=$(awk -v ny="$N_YAW" -v npy="$N_PER_YAW" -v ed="$EPISODE_DURATION_S" -v ov="$PER_EPISODE_OVERHEAD_S" \
        'BEGIN{cond_s=ny*npy*(ed+ov); rot_s=ny*4; overhead_s=120; printf "%d", (cond_s+rot_s+overhead_s)*1.3}')
fi
echo "=== 세션 ${SESSIONS}개, 세션당 yaw ${N_YAW} x 방향당 ${N_PER_YAW} = ${TOTAL_PER_SESSION}조건 (에피소드당 ${EPISODE_DURATION_S}초) ==="
echo "=== 총 예상 조건 수: $((TOTAL_PER_SESSION * SESSIONS)) ==="
echo "=== 세션들을 합친 yaw 해상도: ${COMBINED_YAW_POINTS}지점 (${COMBINED_YAW_STEP}도 간격) ==="
echo "=== 예상 총 소요시간: 약 ${EST_HOURS}시간 (대략적인 추정치) ==="
echo "=== 세션당 강제종료 타임아웃: ${SESSION_TIMEOUT_S}초 ==="

restart_sitl() {
    echo "  [SITL] 기존 프로세스 정리 중 (정상종료 시도)..."
    pkill -TERM -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -TERM -f "gz sim" 2>/dev/null
    pkill -TERM -f "make px4_sitl" 2>/dev/null
    pkill -TERM -f "mavsdk_server" 2>/dev/null
    pkill -TERM -f "wind_gust_sweep.py" 2>/dev/null

    local term_waited=0
    while pgrep -f "px4_sitl_default/bin/px4|gz sim|mavsdk_server|wind_gust_sweep.py" > /dev/null \
          && [[ $term_waited -lt 8 ]]; do
        sleep 1
        term_waited=$((term_waited + 1))
    done

    echo "  [SITL] 강제 정리 (남아있는 프로세스 있으면)..."
    pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "make px4_sitl" 2>/dev/null
    pkill -9 -f "mavsdk_server" 2>/dev/null
    pkill -9 -f "wind_gust_sweep.py" 2>/dev/null
    local leftover
    leftover=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|mavsdk_server")
    if [[ -n "$leftover" ]]; then
        echo "  [SITL] pkill로 안 죽은 프로세스 발견, PID로 직접 정리: $leftover"
        echo "$leftover" | xargs -r kill -9
    fi
    rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
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
for i in $(seq "$START_SESSION" "$SESSIONS"); do
    echo ""
    echo "############################################################"
    echo "# 세션 ${i}/${SESSIONS}"
    echo "############################################################"

    session_seed=$((SEED_BASE + i * 1000))
    yaw_offset=$(awk -v n="$N_YAW" -v s="$SESSIONS" -v idx="$((i - 1))" \
        'BEGIN{printf "%.3f", 360.0/n/s*idx}')

    session_ok=0
    for attempt in $(seq 1 "$MAX_RETRIES"); do
        restart_sitl
        echo "  [수집] 시작 (seed=${session_seed}, yaw-offset=${yaw_offset}도, 시도 ${attempt}/${MAX_RETRIES}, 타임아웃 ${SESSION_TIMEOUT_S}초)"
        if (cd "$PROJECT_DIR/sim_scripts/data_collection" && \
            timeout -k 15 "${SESSION_TIMEOUT_S}s" \
            "$PX4SIM_PYTHON" -u wind_gust_sweep.py \
                --n-yaw "$N_YAW" --n-per-yaw "$N_PER_YAW" \
                --episode-duration "$EPISODE_DURATION_S" \
                --yaw-offset "$yaw_offset" --seed "$session_seed"); then
            echo "  [수집] 세션 ${i} 완료 (시도 ${attempt}/${MAX_RETRIES})"
            session_ok=1
            break
        else
            echo "  [수집] 세션 ${i} 시도 ${attempt}/${MAX_RETRIES} 실패 (exit code $?)"
        fi
    done
    if [[ "$session_ok" -ne 1 ]]; then
        echo "  [수집] 세션 ${i} 최종 실패 (${MAX_RETRIES}회 모두 실패) - 다음 세션으로 계속 진행"
        FAILED_SESSIONS=$((FAILED_SESSIONS + 1))
    fi
done

echo ""
echo "############################################################"
echo "# 전체 완료: ${SESSIONS}개 세션 중 $((SESSIONS - FAILED_SESSIONS))개 성공, ${FAILED_SESSIONS}개 실패"
echo "# 결과 CSV: ${PROJECT_DIR}/logs/wind_gust_*.csv"
echo "############################################################"

echo "  [정리] 마지막 SITL 종료 중..."
pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
