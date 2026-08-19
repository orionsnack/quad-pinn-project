#!/bin/bash
# RAMP-Net 파라메트릭 불확실성 주입(k_noise_sigma) 스윕. EXPERIMENTS.md 12-15/12-22절
# 참고 - 항력계수 k에 학습 중 노이즈를 섞어 gust 같은 급변 상황에 더 강인해지는지 확인.
#
# 배포용 wind_estimator.pt를 안 건드리도록 주의: train_wind_estimator.py는 --kfold
# 진단과 별개로 항상 단일 80/20 분할로 배포용 모델을 재학습·저장함 - sigma를 여러 개
# 돌리면 매번 이 파일을 덮어씀. 그래서 매 sigma 실행 전 원본으로 복원하고, 실행 후
# 결과를 sigma별 파일명으로 따로 저장 - 스윕이 끝나도 배포된 파일은 원본 그대로 유지됨
# (어떤 sigma를 실제로 배포할지는 결과 보고 사람이 따로 결정).
#
# 사용법: ./sweep_k_noise_sigma.sh (offline_training/ 안에서 실행)

set -uo pipefail
cd "$(dirname "$0")"

SIGMAS=(0.0 0.02 0.05 0.1)
CSVS=(../logs/wind_random_dither_*.csv ../logs/wind_gust_*.csv)

if [[ ! -f wind_estimator.pt ]]; then
    echo "wind_estimator.pt 없음 - 배포된 모델이 있는 상태에서 실행할 것"
    exit 1
fi
cp wind_estimator.pt wind_estimator.pt.orig_backup

RUN_TS=$(date +%Y%m%d_%H%M%S)
mkdir -p "sweep_${RUN_TS}"

for s in "${SIGMAS[@]}"; do
    echo ""
    echo "############################################################"
    echo "# k_noise_sigma=${s}  ($(date))"
    echo "############################################################"
    cp wind_estimator.pt.orig_backup wind_estimator.pt   # 항상 원본에서 시작

    python train_wind_estimator.py "${CSVS[@]}" \
        --kfold 5 --k-noise-sigma "$s" \
        > "sweep_${RUN_TS}/sigma_${s}.log" 2>&1
    status=$?

    if [[ $status -eq 0 ]]; then
        cp wind_estimator.pt "sweep_${RUN_TS}/wind_estimator_sigma_${s}.pt"
        tail -20 "sweep_${RUN_TS}/sigma_${s}.log"
    else
        echo "  [실패] sigma=${s} (exit ${status}) - 로그: sweep_${RUN_TS}/sigma_${s}.log"
    fi
done

cp wind_estimator.pt.orig_backup wind_estimator.pt   # 배포 파일은 원본 그대로 복원
rm -f wind_estimator.pt.orig_backup

echo ""
echo "############################################################"
echo "# 스윕 완료 - 결과: offline_training/sweep_${RUN_TS}/"
echo "# 배포된 wind_estimator.pt는 원본 그대로 유지됨 (sigma별 결과는 별도 파일)"
echo "############################################################"
