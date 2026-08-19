"""
PINN 기반 외란(바람벡터 + 회전 외란토크) 추정 모델 학습.

목표: 최근 상태 이력(수평속도, roll/pitch, 각속도)으로부터 현재 바람벡터
(wind_vx, wind_vy)와 회전 외란토크(tau_dist_x/y/z)를 함께 추정하는 작은 네트워크를 학습.

**물리 방정식/모델 구조/하이퍼파라미터는 `wind_pinn_model.py`로 옮겨졌음** - 항력식,
회전운동방정식, WINDOW/HIDDEN/LAMBDA_PHYSICS/LAMBDA_ROT 등을 고치고 싶으면 그 파일만
보면 됨. 이 파일은 데이터 로딩(build_windows), 학습 루프(train_one_split),
k-fold(run_kfold), CLI(main)만 담당.

손실 구성 (2026-08-11, 회전 항 추가):
  1) data_loss (병진만): 지도학습 loss. wind_random_sweep.py는 gz topic으로 바람을
     직접 설정했으므로 정답 바람벡터를 100% 정확히 앎 -> MSE. 모델 출력 5개 중
     앞 2개(wind_vx, wind_vy)에만 해당.
  2) physics_loss (병진): 추정된 바람으로 계산한 공기저항 가속도가 실측(유한차분)
     가속도와 일치해야 한다는 물리 제약 (wind_pinn_model.physics_residual 참고).
  3) physics_loss_rot (회전, 신규): 병진과 달리 회전 쪽엔 별도 정답 라벨이 없음 -
     tau_disturbance = J*omega_dot(실측) - tau_motor(계산)로 그 자리에서 바로 구해지는
     값이라 물리식 자체가 라벨 역할을 겸함. 모델 출력 뒤 3개(tau_dist_x/y/z)가 이 값을
     추정하도록, J*omega_dot_pred = tau_motor + tau_dist_pred가 실측 omega_dot과
     맞도록 학습 (wind_pinn_model.physics_residual_rotation 참고).

주의(중요) 1 - yaw 종속성: feature로 쓰는 roll/pitch는 기체(body) 좌표계라, 같은
바람이라도 기수 방향(yaw)에 따라 값이 달라짐. vn/ve는 이미 관성(NED) 좌표계라
문제없지만 roll/pitch는 그대로 쓰면 "학습 당시의 yaw에서만 통하는" 모델이 됨.
`yaw_decompose()`로 4개 항을 만들어 신경망이 정확한 결합 방식을 데이터로부터 배우게
함 (자세한 시행착오는 EXPERIMENTS.md 12-10절). 각속도(wx/wy/wz)는 이미 body frame
벡터라 이 문제가 재발하지 않아 변환 없이 그대로 씀.

평가 방법(중요): 조건 수가 적을 때(지금 최대 백여 개) 한 번의 무작위 80/20 분할만으로
"좋아졌다/나빠졌다"를 판단하면, 그 분할에 어떤 조건이 뽑혔는지에 따라 결과가 크게
흔들림. --kfold N을 주면 조건을 N등분해서 N번 학습/검증을 반복하고 평균±표준편차로
훨씬 안정적인 성능 추정치를 보여줌(모델 저장은 안 하는 순수 진단용). 이후 항상 마지막에
단일 80/20 분할로 한 번 더 학습해서 실제 배포용 모델(wind_estimator.pt)을 저장함.

사용:
  python train_wind_estimator.py <csv...>              # 배포용 모델만 학습
  python train_wind_estimator.py <csv...> --kfold 5     # 5-fold 진단 + 배포용 모델 학습

주의: 2026-08-11 이전에 모은 CSV(각속도/모터명령 컬럼 없음)는 이 스크립트로 못 씀 -
wind_random_sweep.py / wind_gust_sweep.py로 재수집 필요.
"""

import argparse
import functools
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn

from wind_pinn_model import (
    WINDOW, FEATURES, LAMBDA_PHYSICS, LAMBDA_ROT, EPOCHS, LR, BATCH_SIZE, WEIGHT_DECAY,
    VAL_FRACTION, WindPINN, yaw_decompose, physics_residual, physics_residual_rotation,
)

# 터미널/로그 파일로 리다이렉트했을 때도 실시간으로 진행상황이 보이게 항상 flush
print = functools.partial(print, flush=True)

ACTUATOR_COLS = ["actuator0", "actuator1", "actuator2", "actuator3"]
GYRO_COLS = ["wx_rad_s", "wy_rad_s", "wz_rad_s"]


def build_windows(df: pd.DataFrame):
    """condition_idx별로 시간순 정렬 후, 슬라이딩 윈도우 (X, y, accel, vdrone,
    omega_dot, actuator) 생성."""
    df = df.copy()

    X_list, y_list, acc_list, vdrone_list = [], [], [], []
    omega_dot_list, actuator_list, cond_list = [], [], []

    for cond_idx, g in df.groupby("condition_idx"):
        g = g.sort_values("t_s").reset_index(drop=True)
        t = g["t_s"].to_numpy()
        vn = g["vn_m_s"].to_numpy()
        ve = g["ve_m_s"].to_numpy()
        an = np.gradient(vn, t)  # 유한차분 가속도 (m/s^2)
        ae = np.gradient(ve, t)

        wx = g["wx_rad_s"].to_numpy()
        wy = g["wy_rad_s"].to_numpy()
        wz = g["wz_rad_s"].to_numpy()
        wx_dot = np.gradient(wx, t)  # 유한차분 각가속도 (rad/s^2)
        wy_dot = np.gradient(wy, t)
        wz_dot = np.gradient(wz, t)

        (g["roll_cos_yaw"], g["roll_sin_yaw"],
         g["pitch_cos_yaw"], g["pitch_sin_yaw"]) = yaw_decompose(
            g["roll_deg"].to_numpy(), g["pitch_deg"].to_numpy(), g["yaw_deg"].to_numpy())

        feats = g[FEATURES].to_numpy()
        actuator = g[ACTUATOR_COLS].to_numpy()
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
            omega_dot_list.append([wx_dot[i], wy_dot[i], wz_dot[i]])
            actuator_list.append(actuator[i])
            cond_list.append(cond_idx)

    X = np.asarray(X_list, dtype=np.float32)
    y = np.asarray(y_list, dtype=np.float32)
    acc = np.asarray(acc_list, dtype=np.float32)
    vdrone = np.asarray(vdrone_list, dtype=np.float32)
    omega_dot = np.asarray(omega_dot_list, dtype=np.float32)
    actuator = np.asarray(actuator_list, dtype=np.float32)
    cond = np.asarray(cond_list)
    return X, y, acc, vdrone, omega_dot, actuator, cond


def train_one_split(X, y, acc, vdrone, omega_dot, actuator, train_mask, val_mask, verbose=False,
                     k_noise_sigma=0.0):
    """train_mask/val_mask 기준으로 정규화(train만 기준) 후 학습, best checkpoint 반환.
    best checkpoint는 병진 val_MAE(wind_vx/vy) 기준으로 고름 - 회전 쪽은 정답 라벨이
    없어 같은 방식으로 비교할 기준이 없기 때문(위 모듈 docstring 참고)."""
    X_mean, X_std = X[train_mask].mean(0), X[train_mask].std(0) + 1e-6
    Xn = (X - X_mean) / X_std

    to_t = torch.from_numpy
    X_t, y_t, acc_t, vd_t = to_t(Xn), to_t(y), to_t(acc), to_t(vdrone)
    wdot_t, act_t = to_t(omega_dot), to_t(actuator)

    Xtr, ytr, acctr, vdtr = X_t[train_mask], y_t[train_mask], acc_t[train_mask], vd_t[train_mask]
    wdottr, acttr = wdot_t[train_mask], act_t[train_mask]
    Xva, yva, wdotva, actva = X_t[val_mask], y_t[val_mask], wdot_t[val_mask], act_t[val_mask]

    model = WindPINN(WINDOW, len(FEATURES))
    opt = torch.optim.Adam(model.parameters(), lr=LR, weight_decay=WEIGHT_DECAY)

    best_val_mae = float("inf")
    best_state = None
    best_epoch = -1
    n_train = len(Xtr)
    # early stopping: 고정 EPOCHS 값을 추측하는 대신 실제 수렴 시점에 맞춰 멈춤.
    # 배포된 체크포인트의 best_epoch=187(전체 더더링+gust 데이터 기준)이었던 걸
    # 뒤늦게 확인함 - 데이터셋/sigma마다 수렴 시점이 크게 다를 수 있어 고정값
    # 추측(예: EPOCHS=80)은 위험함. PATIENCE만큼 개선 없으면 중단 (2026-08-20).
    PATIENCE = 50
    epochs_since_improve = 0

    for epoch in range(1, EPOCHS + 1):
        model.train()
        perm = torch.randperm(n_train)
        epoch_data_loss = epoch_physics_loss = epoch_rot_loss = 0.0
        for start in range(0, n_train, BATCH_SIZE):
            idx = perm[start:start + BATCH_SIZE]
            opt.zero_grad()
            pred = model(Xtr[idx])
            wind_pred, tau_dist_pred = pred[:, :2], pred[:, 2:5]

            data_loss = nn.functional.mse_loss(wind_pred, ytr[idx])
            a_pred = physics_residual(model, wind_pred, vdtr[idx], k_noise_sigma=k_noise_sigma)
            physics_loss = nn.functional.mse_loss(a_pred, acctr[idx])

            omega_dot_pred = physics_residual_rotation(tau_dist_pred, acttr[idx])
            rot_loss = nn.functional.mse_loss(omega_dot_pred, wdottr[idx])

            loss = data_loss + LAMBDA_PHYSICS * physics_loss + LAMBDA_ROT * rot_loss
            loss.backward()
            opt.step()
            epoch_data_loss += data_loss.item() * len(idx)
            epoch_physics_loss += physics_loss.item() * len(idx)
            epoch_rot_loss += rot_loss.item() * len(idx)
        data_loss_avg = epoch_data_loss / n_train
        physics_loss_avg = epoch_physics_loss / n_train
        rot_loss_avg = epoch_rot_loss / n_train

        model.eval()
        with torch.no_grad():
            pred_va = model(Xva)
            wind_pred_va = pred_va[:, :2]
            val_mae = (wind_pred_va - yva).abs().mean().item()
        if val_mae < best_val_mae:
            best_val_mae = val_mae
            best_epoch = epoch
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
            epochs_since_improve = 0
        else:
            epochs_since_improve += 1

        if verbose and (epoch % 5 == 0 or epoch == 1):
            print(f"  epoch {epoch:4d}  train_data_loss={data_loss_avg:.4f}  "
                  f"physics_loss={physics_loss_avg:.4f}  rot_loss={rot_loss_avg:.4f}  "
                  f"val_MAE={val_mae:.3f}m/s  k={model.k.item():.4f}", flush=True)

        if epochs_since_improve >= PATIENCE:
            if verbose:
                print(f"  -> {PATIENCE}epoch 동안 개선 없어 조기 종료 (epoch {epoch}, "
                      f"best_epoch={best_epoch})", flush=True)
            break

    model.load_state_dict(best_state)
    model.eval()
    with torch.no_grad():
        pred_va = model(Xva)
        wind_pred_va, tau_dist_pred_va = pred_va[:, :2], pred_va[:, 2:5]
        err = (wind_pred_va - yva).numpy()
        speed_true = np.linalg.norm(yva.numpy(), axis=1)
        speed_pred = np.linalg.norm(wind_pred_va.numpy(), axis=1)

        omega_dot_pred_va = physics_residual_rotation(tau_dist_pred_va, actva)
        rot_residual = (omega_dot_pred_va - wdotva).abs().mean().item()
        tau_dist_norm = tau_dist_pred_va.norm(dim=1).mean().item()

    metrics = {
        "wind_vx_mae": float(np.abs(err[:, 0]).mean()),
        "wind_vy_mae": float(np.abs(err[:, 1]).mean()),
        "speed_mae": float(np.abs(speed_true - speed_pred).mean()),
        "mean_true_speed": float(speed_true.mean()),
        "best_epoch": best_epoch,
        "best_val_mae": best_val_mae,
        "n_val": int(val_mask.sum()),
        "k": model.k.item(),
        # 회전 쪽은 정답 라벨이 없어 "MAE vs 정답"이 아니라 물리잔차(omega_dot 예측오차)로
        # 품질을 봄 - 0에 가까울수록 tau_disturbance 추정이 실측 각가속도를 잘 설명한다는 뜻.
        "rot_omega_dot_residual_mae": float(rot_residual),
        # tau_disturbance 추정치의 평균 크기(N*m) - 무풍 조건이 섞여있다면 이게 너무 크면
        # motor_torque()의 액추에이터 스케일 가정(wind_pinn_model.py 상단 주석 참고)이
        # 잘못됐을 가능성을 의심할 것.
        "tau_dist_mean_norm": float(tau_dist_norm),
    }
    return model, best_state, X_mean, X_std, metrics


def run_kfold(X, y, acc, vdrone, omega_dot, actuator, cond, k, k_noise_sigma=0.0):
    unique_conds = np.unique(cond)
    rng = np.random.RandomState(0)
    rng.shuffle(unique_conds)
    folds = np.array_split(unique_conds, k)

    print(f"\n=== {k}-fold 교차검증 (조건 {len(unique_conds)}개를 {k}등분, 진단용 - 모델 저장 안 함, "
          f"k_noise_sigma={k_noise_sigma}) ===")
    all_metrics = []
    for i, fold_conds in enumerate(folds):
        val_conds = set(fold_conds.tolist())
        val_mask = np.isin(cond, list(val_conds))
        train_mask = ~val_mask
        _, _, _, _, metrics = train_one_split(
            X, y, acc, vdrone, omega_dot, actuator, train_mask, val_mask, verbose=False,
            k_noise_sigma=k_noise_sigma)
        print(f"  fold {i+1}/{k} (검증조건 {len(fold_conds)}개, n_val={metrics['n_val']}): "
              f"wind_vx MAE={metrics['wind_vx_mae']:.3f}  wind_vy MAE={metrics['wind_vy_mae']:.3f}  "
              f"speed MAE={metrics['speed_mae']:.3f}  rot_residual={metrics['rot_omega_dot_residual_mae']:.3f}  "
              f"best_epoch={metrics['best_epoch']}")
        all_metrics.append(metrics)

    print(f"\n--- {k}-fold 평균±표준편차 (단일 분할보다 훨씬 안정적인 성능 추정치) ---")
    for key in ["wind_vx_mae", "wind_vy_mae", "speed_mae", "rot_omega_dot_residual_mae"]:
        vals = [m[key] for m in all_metrics]
        print(f"  {key}: {np.mean(vals):.3f} ± {np.std(vals):.3f}  "
              f"(범위 {min(vals):.3f}~{max(vals):.3f})")
    return all_metrics


def main():
    global EPOCHS
    parser = argparse.ArgumentParser()
    parser.add_argument("csvs", nargs="+", help="wind_random_*.csv / wind_gust_*.csv")
    parser.add_argument("--kfold", type=int, default=0,
                         help="k-fold 교차검증 fold 수 (0이면 끔, 순수 진단용이라 모델 저장 안 함)")
    parser.add_argument("--k-noise-sigma", type=float, default=0.0,
                         help="RAMP-Net 파라메트릭 불확실성 주입 강도 (0=끔, 기존 동작과 동일). "
                              "물리손실 계산에 쓰는 항력계수 k에 상대 노이즈 N(0,sigma)를 섞음 "
                              "(EXPERIMENTS.md 12-15/12-22절)")
    parser.add_argument("--epochs", type=int, default=EPOCHS,
                         help="최대 epoch 수 (기본 400은 회전 결합 데이터에서 과함 - "
                              "12-16절 참고, best_epoch가 보통 30 이전에 나옴). "
                              "체크포인트는 항상 best-val-mae 기준이라 줄여도 품질 손해 없음")
    args = parser.parse_args()
    EPOCHS = args.epochs

    dfs = [pd.read_csv(p) for p in args.csvs]
    # 여러 CSV를 합칠 때 condition_idx가 겹치지 않도록 오프셋
    offset = 0
    for d in dfs:
        d["condition_idx"] += offset
        offset = d["condition_idx"].max() + 1
    df = pd.concat(dfs, ignore_index=True)
    print(f"로드된 행 수: {len(df)}, 조건 수: {df['condition_idx'].nunique()}")

    X, y, acc, vdrone, omega_dot, actuator, cond = build_windows(df)
    print(f"윈도우 샘플 수: {len(X)} (window={WINDOW}, features={len(FEATURES)})")

    if args.kfold and args.kfold >= 2:
        run_kfold(X, y, acc, vdrone, omega_dot, actuator, cond, args.kfold,
                  k_noise_sigma=args.k_noise_sigma)

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
        X, y, acc, vdrone, omega_dot, actuator, train_mask, val_mask, verbose=True,
        k_noise_sigma=args.k_noise_sigma)

    print(f"\n최적 체크포인트: epoch {metrics['best_epoch']}  val_MAE={metrics['best_val_mae']:.3f}m/s")
    print(f"=== 최종 검증(val, 미학습 바람조건 {metrics['n_val']}개, best checkpoint) ===")
    print(f"  wind_vx MAE={metrics['wind_vx_mae']:.3f} m/s  wind_vy MAE={metrics['wind_vy_mae']:.3f} m/s")
    print(f"  풍속(speed) MAE={metrics['speed_mae']:.3f} m/s (평균 실제 풍속={metrics['mean_true_speed']:.2f} m/s)")
    print(f"  학습된 항력계수 k = {metrics['k']:.5f}")
    print(f"  [회전] omega_dot 물리잔차 MAE = {metrics['rot_omega_dot_residual_mae']:.4f} rad/s^2")
    print(f"  [회전] tau_disturbance 평균 크기 = {metrics['tau_dist_mean_norm']:.4f} N*m")

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
