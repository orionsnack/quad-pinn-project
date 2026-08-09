"""
PINN 기반 바람(외란) 벡터 추정 모델 학습.

목표: 최근 상태 이력(수평 속도, roll/pitch, setpoint 대비 위치오차)으로부터
현재 바람 벡터(wind_vx, wind_vy)를 추정하는 작은 네트워크를 학습.

**물리 방정식/모델 구조/하이퍼파라미터는 `wind_pinn_model.py`로 옮겨졌음** - 항력
방정식이나 WINDOW/HIDDEN/LAMBDA_PHYSICS 등을 고치고 싶으면 그 파일만 보면 됨.
이 파일은 데이터 로딩(build_windows), 학습 루프(train_one_split), k-fold(run_kfold),
CLI(main)만 담당.

두 가지 loss를 함께 사용:
  1) data_loss: 지도학습 loss. sim_scripts/wind_random_sweep.py는 gz topic으로
     바람을 직접 설정했으므로 정답 바람벡터를 100% 정확히 알고 있음 -> MSE.
  2) physics_loss: 추정된 바람으로 계산한 공기저항 가속도가 실측(유한차분) 가속도와
     일치해야 한다는 물리 제약(PINN residual, 식은 wind_pinn_model.physics_residual 참고).

주의(중요) 2 - yaw 종속성: feature로 쓰는 roll/pitch는 기체(body) 좌표계라, 같은
바람이라도 기수 방향(yaw)에 따라 값이 달라짐(북풍을 맞을 때 기수가 북쪽이면 주로
pitch로, 동쪽이면 주로 roll로 나타남). vn/ve는 이미 관성(NED) 좌표계라 문제없지만
roll/pitch는 그대로 쓰면 "학습 당시의 yaw에서만 통하는" 모델이 됨.

1차 시도(실패, 이건 확실함): roll/pitch가 yaw에 대해 대칭으로(같은 비중으로) 회전한다고
가정하고 손으로 회전 공식을 유도해서 tilt_north/tilt_east 2개로 축약했었음. 그런데
yaw=0도로 따로 모은 검증 데이터로 확인해보니 val_MAE가 정상 대비 3~4배로 나쁘게
나옴 (wind_yaw_generalization_test.py로 검증, evaluate_checkpoint.py로 확인) - 이
실패 자체는 직접 측정한 확실한 결과.
왜 실패했는지는 불확실함: yaw=92도/0도 데이터를 합쳐 회귀분석해보면 roll/pitch 게인이
비대칭인 것처럼 보이는 패턴이 나오긴 하는데, yaw 표본이 딱 2종류뿐이라 이 회귀 자체의
신뢰도가 낮음 - 다른 원인이었을 가능성도 있어서 "왜"는 확정하지 않음.

2차 시도(현재): 정확한 결합 공식을 손으로 다시 유도하는 대신, `roll_deg*cos(yaw)`,
`roll_deg*sin(yaw)`, `pitch_deg*cos(yaw)`, `pitch_deg*sin(yaw)` 4개 항을 그대로
feature로 주고("물리적으로 그럴듯한 후보 기저"만 제공), roll/pitch 축의 게인이 비대칭
이어도 상관없이(원인이 뭐였든) 정확한 결합 방식은 신경망의 hidden layer가 (yaw가
다양한) 데이터로부터 직접 배우게 함 (`yaw_decompose()`). 이러려면 학습 데이터의
yaw 자체가 다양해야 하므로, `wind_random_sweep.py`가 조건마다 yaw도 같이 그리드로
바꾸도록 확장함.

평가 방법(중요): 조건 수가 적을 때(지금 최대 백여 개) 한 번의 무작위 80/20 분할만으로
"좋아졌다/나빠졌다"를 판단하면, 그 분할에 어떤 조건이 뽑혔는지에 따라 결과가 크게
흔들림 (실제로 재학습마다 val_MAE가 들쭉날쭉했음). --kfold N을 주면 조건을 N등분해서
N번 학습/검증을 반복하고 평균±표준편차로 훨씬 안정적인 성능 추정치를 보여줌
(모델 저장은 안 하는 순수 진단용). 이후 항상 마지막에 단일 80/20 분할로 한 번 더
학습해서 실제 배포용 모델(wind_estimator.pt)을 저장함.

사용:
  python train_wind_estimator.py <csv...>              # 배포용 모델만 학습
  python train_wind_estimator.py <csv...> --kfold 5     # 5-fold 진단 + 배포용 모델 학습
"""

import argparse
import functools
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn

from wind_pinn_model import (
    WINDOW, FEATURES, LAMBDA_PHYSICS, EPOCHS, LR, BATCH_SIZE, WEIGHT_DECAY,
    VAL_FRACTION, WindPINN, yaw_decompose, physics_residual,
)

# 터미널/로그 파일로 리다이렉트했을 때도 실시간으로 진행상황이 보이게 항상 flush
print = functools.partial(print, flush=True)


def build_windows(df: pd.DataFrame):
    """condition_idx별로 시간순 정렬 후, 슬라이딩 윈도우 (X, y, accel, v_drone) 생성."""
    df = df.copy()

    X_list, y_list, acc_list, vdrone_list, cond_list = [], [], [], [], []

    for cond_idx, g in df.groupby("condition_idx"):
        g = g.sort_values("t_s").reset_index(drop=True)
        t = g["t_s"].to_numpy()
        vn = g["vn_m_s"].to_numpy()
        ve = g["ve_m_s"].to_numpy()
        an = np.gradient(vn, t)  # 유한차분 가속도 (m/s^2)
        ae = np.gradient(ve, t)

        (g["roll_cos_yaw"], g["roll_sin_yaw"],
         g["pitch_cos_yaw"], g["pitch_sin_yaw"]) = yaw_decompose(
            g["roll_deg"].to_numpy(), g["pitch_deg"].to_numpy(), g["yaw_deg"].to_numpy())

        feats = g[FEATURES].to_numpy()
        # 행마다 라벨을 따로 씀 (iloc[0] 고정값이 아님) - wind_random_sweep.py처럼
        # 트라이얼 내내 바람이 일정한 데이터는 모든 행이 같은 값이라 결과가 똑같고,
        # wind_gust_sweep.py처럼 시간에 따라 바람이 변하는 데이터도 그대로 지원됨.
        wind_vx = g["wind_vx_m_s"].to_numpy()
        wind_vy = g["wind_vy_m_s"].to_numpy()

        n = len(g)
        for i in range(WINDOW, n):
            window_feats = feats[i - WINDOW:i].flatten()
            X_list.append(window_feats)
            y_list.append([wind_vx[i], wind_vy[i]])
            acc_list.append([an[i], ae[i]])
            vdrone_list.append([vn[i], ve[i]])
            cond_list.append(cond_idx)

    X = np.asarray(X_list, dtype=np.float32)
    y = np.asarray(y_list, dtype=np.float32)
    acc = np.asarray(acc_list, dtype=np.float32)
    vdrone = np.asarray(vdrone_list, dtype=np.float32)
    cond = np.asarray(cond_list)
    return X, y, acc, vdrone, cond


def train_one_split(X, y, acc, vdrone, train_mask, val_mask, verbose=False):
    """train_mask/val_mask 기준으로 정규화(train만 기준) 후 학습, best checkpoint 반환."""
    X_mean, X_std = X[train_mask].mean(0), X[train_mask].std(0) + 1e-6
    Xn = (X - X_mean) / X_std

    to_t = torch.from_numpy
    X_t, y_t, acc_t, vd_t = to_t(Xn), to_t(y), to_t(acc), to_t(vdrone)

    Xtr, ytr, acctr, vdtr = X_t[train_mask], y_t[train_mask], acc_t[train_mask], vd_t[train_mask]
    Xva, yva = X_t[val_mask], y_t[val_mask]

    model = WindPINN(WINDOW, len(FEATURES))
    opt = torch.optim.Adam(model.parameters(), lr=LR, weight_decay=WEIGHT_DECAY)

    best_val_mae = float("inf")
    best_state = None
    best_epoch = -1
    n_train = len(Xtr)

    for epoch in range(1, EPOCHS + 1):
        model.train()
        perm = torch.randperm(n_train)
        epoch_data_loss = epoch_physics_loss = 0.0
        for start in range(0, n_train, BATCH_SIZE):
            idx = perm[start:start + BATCH_SIZE]
            opt.zero_grad()
            pred = model(Xtr[idx])
            data_loss = nn.functional.mse_loss(pred, ytr[idx])
            a_pred = physics_residual(model, pred, vdtr[idx])
            physics_loss = nn.functional.mse_loss(a_pred, acctr[idx])
            loss = data_loss + LAMBDA_PHYSICS * physics_loss
            loss.backward()
            opt.step()
            epoch_data_loss += data_loss.item() * len(idx)
            epoch_physics_loss += physics_loss.item() * len(idx)
        data_loss_avg = epoch_data_loss / n_train
        physics_loss_avg = epoch_physics_loss / n_train

        model.eval()
        with torch.no_grad():
            pred_va = model(Xva)
            val_mae = (pred_va - yva).abs().mean().item()
        if val_mae < best_val_mae:
            best_val_mae = val_mae
            best_epoch = epoch
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

        if verbose and (epoch % 5 == 0 or epoch == 1):
            print(f"  epoch {epoch:4d}  train_data_loss={data_loss_avg:.4f}  "
                  f"physics_loss={physics_loss_avg:.4f}  val_MAE={val_mae:.3f}m/s  "
                  f"k={model.k.item():.4f}", flush=True)

    model.load_state_dict(best_state)
    model.eval()
    with torch.no_grad():
        pred_va = model(Xva)
        err = (pred_va - yva).numpy()
        speed_true = np.linalg.norm(yva.numpy(), axis=1)
        speed_pred = np.linalg.norm(pred_va.numpy(), axis=1)

    metrics = {
        "wind_vx_mae": float(np.abs(err[:, 0]).mean()),
        "wind_vy_mae": float(np.abs(err[:, 1]).mean()),
        "speed_mae": float(np.abs(speed_true - speed_pred).mean()),
        "mean_true_speed": float(speed_true.mean()),
        "best_epoch": best_epoch,
        "best_val_mae": best_val_mae,
        "n_val": int(val_mask.sum()),
        "k": model.k.item(),
    }
    return model, best_state, X_mean, X_std, metrics


def run_kfold(X, y, acc, vdrone, cond, k):
    unique_conds = np.unique(cond)
    rng = np.random.RandomState(0)
    rng.shuffle(unique_conds)
    folds = np.array_split(unique_conds, k)

    print(f"\n=== {k}-fold 교차검증 (조건 {len(unique_conds)}개를 {k}등분, 진단용 - 모델 저장 안 함) ===")
    all_metrics = []
    for i, fold_conds in enumerate(folds):
        val_conds = set(fold_conds.tolist())
        val_mask = np.isin(cond, list(val_conds))
        train_mask = ~val_mask
        _, _, _, _, metrics = train_one_split(X, y, acc, vdrone, train_mask, val_mask, verbose=False)
        print(f"  fold {i+1}/{k} (검증조건 {len(fold_conds)}개, n_val={metrics['n_val']}): "
              f"wind_vx MAE={metrics['wind_vx_mae']:.3f}  wind_vy MAE={metrics['wind_vy_mae']:.3f}  "
              f"speed MAE={metrics['speed_mae']:.3f}  best_epoch={metrics['best_epoch']}")
        all_metrics.append(metrics)

    print(f"\n--- {k}-fold 평균±표준편차 (단일 분할보다 훨씬 안정적인 성능 추정치) ---")
    for key in ["wind_vx_mae", "wind_vy_mae", "speed_mae"]:
        vals = [m[key] for m in all_metrics]
        print(f"  {key}: {np.mean(vals):.3f} ± {np.std(vals):.3f}  "
              f"(범위 {min(vals):.3f}~{max(vals):.3f})")
    return all_metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csvs", nargs="+", help="wind_random_*.csv / wind_gust_*.csv")
    parser.add_argument("--kfold", type=int, default=0,
                         help="k-fold 교차검증 fold 수 (0이면 끔, 순수 진단용이라 모델 저장 안 함)")
    args = parser.parse_args()

    dfs = [pd.read_csv(p) for p in args.csvs]
    # 여러 CSV를 합칠 때 condition_idx가 겹치지 않도록 오프셋
    offset = 0
    for d in dfs:
        d["condition_idx"] += offset
        offset = d["condition_idx"].max() + 1
    df = pd.concat(dfs, ignore_index=True)
    print(f"로드된 행 수: {len(df)}, 조건 수: {df['condition_idx'].nunique()}")

    X, y, acc, vdrone, cond = build_windows(df)
    print(f"윈도우 샘플 수: {len(X)} (window={WINDOW}, features={len(FEATURES)})")

    if args.kfold and args.kfold >= 2:
        run_kfold(X, y, acc, vdrone, cond, args.kfold)

    # --- 배포용 모델: 단일 80/20 분할로 학습 + 저장 ---
    print(f"\n=== 배포용 모델 학습 (단일 {int((1-VAL_FRACTION)*100)}/{int(VAL_FRACTION*100)} 분할) ===")
    unique_conds = np.unique(cond)
    rng = np.random.RandomState(0)
    rng.shuffle(unique_conds)
    n_val = max(1, int(len(unique_conds) * VAL_FRACTION))
    val_conds = set(unique_conds[:n_val].tolist())
    val_mask = np.isin(cond, list(val_conds))
    train_mask = ~val_mask
    print(f"train조건={len(unique_conds)-n_val}  val조건={n_val}")

    model, best_state, X_mean, X_std, metrics = train_one_split(
        X, y, acc, vdrone, train_mask, val_mask, verbose=True)

    print(f"\n최적 체크포인트: epoch {metrics['best_epoch']}  val_MAE={metrics['best_val_mae']:.3f}m/s")
    print(f"=== 최종 검증(val, 미학습 바람조건 {metrics['n_val']}개, best checkpoint) ===")
    print(f"  wind_vx MAE={metrics['wind_vx_mae']:.3f} m/s  wind_vy MAE={metrics['wind_vy_mae']:.3f} m/s")
    print(f"  풍속(speed) MAE={metrics['speed_mae']:.3f} m/s (평균 실제 풍속={metrics['mean_true_speed']:.2f} m/s)")
    print(f"  학습된 항력계수 k = {metrics['k']:.5f}")

    out_dir = Path(__file__).parent
    torch.save({
        "model_state": best_state,
        "X_mean": X_mean, "X_std": X_std,
        "window": WINDOW, "features": FEATURES,
        "best_epoch": metrics["best_epoch"], "best_val_mae": metrics["best_val_mae"],
    }, out_dir / "wind_estimator.pt")
    print(f"\n모델 저장됨: {out_dir / 'wind_estimator.pt'}")


if __name__ == "__main__":
    main()
