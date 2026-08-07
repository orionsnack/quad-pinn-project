"""
저장된 wind_estimator.pt 체크포인트를, 학습에 안 쓴 새 CSV에 대해 평가.

주 용도: yaw 일반화 검증 - wind_yaw_generalization_test.py로 학습 데이터와 다른
yaw에서 모은 CSV를 이 스크립트로 평가해서, tilt_north/tilt_east feature(yaw 회전)
덕분에 다른 방향에서도 성능이 유지되는지 확인. 일반 held-out val_MAE와 직접 비교
가능한 동일 지표(wind_vx/vy MAE, speed MAE)를 출력함.

사용:
  python evaluate_checkpoint.py ../logs/wind_yawtest_0deg_TIMESTAMP.csv
  python evaluate_checkpoint.py <csv...> --model wind_estimator.pt
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import torch

from train_wind_estimator import WindPINN, build_windows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csvs", nargs="+")
    parser.add_argument("--model", default="wind_estimator.pt")
    args = parser.parse_args()

    ckpt = torch.load(args.model, map_location="cpu", weights_only=False)
    print(f"체크포인트: {args.model}")
    print(f"  window={ckpt['window']}  features={ckpt['features']}  "
          f"학습시 best_epoch={ckpt['best_epoch']}  학습시 best_val_mae={ckpt['best_val_mae']:.3f}m/s")

    model = WindPINN(ckpt["window"], len(ckpt["features"]))
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    X_mean = torch.tensor(ckpt["X_mean"], dtype=torch.float32)
    X_std = torch.tensor(ckpt["X_std"], dtype=torch.float32)

    dfs = [pd.read_csv(p) for p in args.csvs]
    offset = 0
    for d in dfs:
        d["condition_idx"] += offset
        offset = d["condition_idx"].max() + 1
    df = pd.concat(dfs, ignore_index=True)
    print(f"\n평가 데이터: {args.csvs}")
    print(f"  행 수: {len(df)}, 조건 수: {df['condition_idx'].nunique()}, "
          f"yaw 범위: {df['yaw_deg'].min():.1f}~{df['yaw_deg'].max():.1f}도")

    X, y, acc, vdrone, cond = build_windows(df)
    print(f"  윈도우 샘플 수: {len(X)}")

    Xn = (torch.from_numpy(X) - X_mean) / X_std
    with torch.no_grad():
        pred = model(Xn)
    err = (pred.numpy() - y)
    speed_true = np.linalg.norm(y, axis=1)
    speed_pred = np.linalg.norm(pred.numpy(), axis=1)

    print(f"\n=== 평가 결과 ===")
    print(f"  wind_vx MAE = {np.abs(err[:, 0]).mean():.3f} m/s")
    print(f"  wind_vy MAE = {np.abs(err[:, 1]).mean():.3f} m/s")
    print(f"  풍속(speed) MAE = {np.abs(speed_true - speed_pred).mean():.3f} m/s "
          f"(평균 실제 풍속={speed_true.mean():.2f} m/s)")


if __name__ == "__main__":
    main()
