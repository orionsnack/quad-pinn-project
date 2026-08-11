#!/bin/bash
# wind_random_sweep.py를 여러 세션으로 나눠서 반복 실행하며, 세션 사이마다 SITL을
# 완전히 재시작하는 오케스트레이션 스크립트.
#
# 왜 필요한가: SITL을 1시간 넘게 안 끄고 돌린 세션에서 위치 추정이 이상해 보이는
# 결과를 한 번 관측한 적 있음(setup_guide.md 트러블슈팅 참고) - 다만 "장시간 실행이 진짜
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
PX4SIM_PYTHON="$HOME/miniconda3/envs/px4sim/bin/python"
PLOT_PYTHON="$HOME/miniconda3/bin/python3"   # matplotlib/pandas 있는 base 환경 (px4sim/pinn_train엔 없음)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSIONS=4
N_YAW=24
N_PER_YAW=10
TRIAL_DURATION=8
SEED_BASE=42
BOOT_WAIT_S=15
START_SESSION=1
MAX_RETRIES=3
PLOT_ENABLED=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sessions) SESSIONS="$2"; shift 2 ;;
        --n-yaw) N_YAW="$2"; shift 2 ;;
        --n-per-yaw) N_PER_YAW="$2"; shift 2 ;;
        --trial-duration) TRIAL_DURATION="$2"; shift 2 ;;
        --seed-base) SEED_BASE="$2"; shift 2 ;;
        --start-session) START_SESSION="$2"; shift 2 ;;
        --no-plot) PLOT_ENABLED=0; shift ;;
        *) echo "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

TOTAL_PER_SESSION=$((N_YAW * N_PER_YAW))
COMBINED_YAW_POINTS=$((N_YAW * SESSIONS))
COMBINED_YAW_STEP=$(awk -v n="$COMBINED_YAW_POINTS" 'BEGIN{printf "%.2f", 360.0/n}')
# 대략적인 총 소요시간 추정치 (조건시간 + yaw회전 + SITL재시작/이착륙 오버헤드 세션당 ~2분 가정)
EST_HOURS=$(awk -v ny="$N_YAW" -v npy="$N_PER_YAW" -v td="$TRIAL_DURATION" -v s="$SESSIONS" \
    'BEGIN{cond_s=ny*npy*(td+2); rot_s=ny*4; overhead_s=120; total=(cond_s+rot_s+overhead_s)*s; printf "%.1f", total/3600}')
# 세션 하나가 이 시간(초)을 넘기면 뭔가 멈춘 것으로 보고 강제 종료 후 재시도함
# (기대 소요시간의 1.3배 - asyncio 내부 타임아웃이 안 먹는 경우에 대비한 OS 레벨 안전장치)
SESSION_TIMEOUT_S=$(awk -v ny="$N_YAW" -v npy="$N_PER_YAW" -v td="$TRIAL_DURATION" \
    'BEGIN{cond_s=ny*npy*(td+2); rot_s=ny*4; overhead_s=120; printf "%d", (cond_s+rot_s+overhead_s)*1.3}')
echo "=== 세션 ${SESSIONS}개, 세션당 yaw ${N_YAW} x 방향당 ${N_PER_YAW} = ${TOTAL_PER_SESSION}조건 (조건당 관측 ${TRIAL_DURATION}초) ==="
echo "=== 총 예상 조건 수: $((TOTAL_PER_SESSION * SESSIONS)) ==="
echo "=== 세션들을 합친 yaw 해상도: ${COMBINED_YAW_POINTS}지점 (${COMBINED_YAW_STEP}도 간격) ==="
echo "=== 예상 총 소요시간: 약 ${EST_HOURS}시간 (대략적인 추정치) ==="
echo "=== 세션당 강제종료 타임아웃: ${SESSION_TIMEOUT_S}초 ==="

# 이번 실행에서 나오는 PNG를 전부 시간+조건명 폴더 하나에 모음 (여러 번 실행해도 안 섞이게)
RUN_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FIGURES_RUN_DIR="$PROJECT_DIR/figures/wind_random_${RUN_TIMESTAMP}_n${N_YAW}x${N_PER_YAW}_td${TRIAL_DURATION}"
if [[ "$PLOT_ENABLED" -eq 1 ]]; then
    mkdir -p "$FIGURES_RUN_DIR"
    echo "=== 그래프 저장 폴더: ${FIGURES_RUN_DIR} ==="
fi
SESSION_CSVS=()
SESSION_PNGS=()

restart_sitl() {
    echo "  [SITL] 기존 프로세스 정리 중 (정상종료 시도)..."
    pkill -TERM -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -TERM -f "gz sim" 2>/dev/null
    pkill -TERM -f "make px4_sitl" 2>/dev/null
    pkill -TERM -f "mavsdk_server" 2>/dev/null
    pkill -TERM -f "wind_random_sweep.py" 2>/dev/null

    local term_waited=0
    while pgrep -f "px4_sitl_default/bin/px4|gz sim|mavsdk_server|wind_random_sweep.py" > /dev/null \
          && [[ $term_waited -lt 8 ]]; do
        sleep 1
        term_waited=$((term_waited + 1))
    done

    echo "  [SITL] 강제 정리 (남아있는 프로세스 있으면)..."
    pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "make px4_sitl" 2>/dev/null
    pkill -9 -f "mavsdk_server" 2>/dev/null
    pkill -9 -f "wind_random_sweep.py" 2>/dev/null
    # pkill -f 패턴 매칭이 이 프로세스는 가끔 못 죽이는 걸 실제로 겪음(2026-08-11) ->
    # 남은 게 있으면 정확한 PID로 한 번 더 확인 후 직접 kill
    local leftover
    leftover=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|mavsdk_server")
    if [[ -n "$leftover" ]]; then
        echo "  [SITL] pkill로 안 죽은 프로세스 발견, PID로 직접 정리: $leftover"
        echo "$leftover" | xargs -r kill -9
    fi
    # PX4 인스턴스 락/소켓이 남으면 다음 인스턴스가 "already running"으로 오작동하며
    # GPS/EKF2가 영영 준비 안 되는 문제를 실제로 겪음(2026-08-11, ~1시간 삽질) -
    # kill -9로 죽이면 정상 종료 핸들러가 안 돌아서 이 파일들이 안 지워지고 남는 게 원인
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
    # 세션마다 그리드를 (360/N_YAW/SESSIONS)*세션순번만큼 밀어서, 세션들을 합쳤을 때
    # 촘촘한 그리드가 되도록 함 (파일 상단 주석 참고)
    yaw_offset=$(awk -v n="$N_YAW" -v s="$SESSIONS" -v idx="$((i - 1))" \
        'BEGIN{printf "%.3f", 360.0/n/s*idx}')

    session_ok=0
    for attempt in $(seq 1 "$MAX_RETRIES"); do
        restart_sitl
        echo "  [수집] 시작 (seed=${session_seed}, yaw-offset=${yaw_offset}도, 시도 ${attempt}/${MAX_RETRIES}, 타임아웃 ${SESSION_TIMEOUT_S}초)"
        if (cd "$PROJECT_DIR/sim_scripts/data_collection" && \
            timeout -k 15 "${SESSION_TIMEOUT_S}s" \
            "$PX4SIM_PYTHON" -u wind_random_sweep.py \
                --n-yaw "$N_YAW" --n-per-yaw "$N_PER_YAW" \
                --trial-duration "$TRIAL_DURATION" \
                --yaw-offset "$yaw_offset" --seed "$session_seed"); then
            echo "  [수집] 세션 ${i} 완료 (시도 ${attempt}/${MAX_RETRIES})"
            session_ok=1
            if [[ "$PLOT_ENABLED" -eq 1 ]]; then
                latest_csv=$(ls -t "$PROJECT_DIR"/logs/wind_random_*.csv 2>/dev/null | head -1)
                if [[ -n "$latest_csv" ]]; then
                    echo "  [그래프] ${latest_csv} 요약 PNG 생성 중..."
                    if "$PLOT_PYTHON" "$SCRIPT_DIR/plot_session_grid.py" "$latest_csv" \
                            --out-dir "$FIGURES_RUN_DIR" 2>&1 | sed 's/^/    /'; then
                        SESSION_CSVS+=("$latest_csv")
                        SESSION_PNGS+=("$FIGURES_RUN_DIR/session_$(basename "${latest_csv%.csv}").png")
                    else
                        echo "  [경고] PNG 생성 실패 (수집된 CSV 데이터는 정상이니 나중에 수동으로 다시 그리면 됨)"
                    fi
                fi
            fi
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
echo "# 결과 CSV: ${PROJECT_DIR}/logs/wind_random_*.csv"
echo "############################################################"

if [[ "$PLOT_ENABLED" -eq 1 && "${#SESSION_CSVS[@]}" -gt 0 ]]; then
    echo ""
    echo "  [그래프] 세션 ${#SESSION_CSVS[@]}개 통합 요약(모자이크+전체 겹침) 생성 중..."
    if ! "$PLOT_PYTHON" "$SCRIPT_DIR/plot_combined_summary.py" \
            --pngs "${SESSION_PNGS[@]}" \
            --csvs "${SESSION_CSVS[@]}" \
            --out-dir "$FIGURES_RUN_DIR" 2>&1 | sed 's/^/    /'; then
        echo "  [경고] 통합 요약 PNG 생성 실패 (세션별 PNG는 이미 저장돼 있음)"
    fi
    echo "  [그래프] 전체 결과: ${FIGURES_RUN_DIR}"
fi

echo "  [정리] 마지막 SITL 종료 중..."
pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
