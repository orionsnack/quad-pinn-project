#!/bin/bash
# run_yaw_collection_sessions.sh가 멈췄는지(hang) 감시하는 워치독.
# "진행 신호(heartbeat)"로 collection_run.log의 수정시각과 가장 최근
# wind_random_*.csv의 수정시각 중 더 최근 것을 씀 - 세션 시작 시점(로그에 찍힘)과
# 실제 비행 중 CSV flush(조건마다, wind_random_sweep.py의 csv_file.flush()) 둘 다를 커버함.
#
# 사용법:
#   ./watch_collection.sh
#   ./watch_collection.sh --log ../../logs/collection_run.log --stall-min 5 --interval-s 60

LOG_FILE="../../logs/collection_run.log"
LOGS_DIR="../../logs"
STALL_MIN=5
INTERVAL_S=60
PROC_PATTERN="run_yaw_collection_sessions.sh"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --log) LOG_FILE="$2"; shift 2 ;;
        --logs-dir) LOGS_DIR="$2"; shift 2 ;;
        --stall-min) STALL_MIN="$2"; shift 2 ;;
        --interval-s) INTERVAL_S="$2"; shift 2 ;;
        *) echo "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

STALL_S=$((STALL_MIN * 60))

echo "=== 워치독 시작: ${STALL_MIN}분 이상 진행 신호 없으면 경고 (매 ${INTERVAL_S}초 체크) ==="

while true; do
    if ! pgrep -f "$PROC_PATTERN" > /dev/null; then
        echo ""
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 수집 스크립트 프로세스가 더 이상 없음."
        if grep -q "전체 완료" "$LOG_FILE" 2>/dev/null; then
            echo "-> '전체 완료' 메시지 확인됨. 정상 종료로 판단, 워치독 종료."
        else
            echo "-> '전체 완료' 메시지가 없음. 비정상 종료 가능성 - ${LOG_FILE} 확인 필요."
            echo -e "\a"
        fi
        break
    fi

    log_mtime=0
    [[ -f "$LOG_FILE" ]] && log_mtime=$(stat -c %Y "$LOG_FILE" 2>/dev/null || echo 0)

    latest_csv=$(ls -t "$LOGS_DIR"/wind_random_*.csv 2>/dev/null | head -1)
    csv_mtime=0
    [[ -n "$latest_csv" ]] && csv_mtime=$(stat -c %Y "$latest_csv" 2>/dev/null || echo 0)

    heartbeat=$((log_mtime > csv_mtime ? log_mtime : csv_mtime))
    now=$(date +%s)
    age=$((now - heartbeat))
    now_str=$(date '+%H:%M:%S')

    if (( age > STALL_S )); then
        echo "[$now_str] 경고: ${age}초(${STALL_MIN}분 이상)째 진행 신호 없음 - 멈춘 것으로 의심됨."
        echo -e "\a"
    else
        echo "[$now_str] 정상 진행 중 (마지막 신호 ${age}초 전)"
    fi

    sleep "$INTERVAL_S"
done
