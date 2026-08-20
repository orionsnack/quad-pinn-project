"""
wind_transition_sweep.py로 모은 데이터에서, "전환(regime 변화) 후 경과시간"에 따라
풍속 추정 오차가 어떻게 변하는지 분석. 지금까지 학습/평가 데이터엔 전환 순간
자체가 없었기 때문에(모든 트라이얼이 한 가지 성격으로 일정, WIND_SETTLE_S로
온셋 직후도 제외) 이건 순수 조사용 스크립트 - 학습 파이프라인에 편입하는 게
목적이 아님.

사용: python analyze_transition_error.py ../logs/wind_transition_TIMESTAMP.csv
"""

import argparse

import numpy as np
import pandas as pd
import torch

from wind_pinn_model import WindPINN, WINDOW, FEATURES, yaw_decompose


def build_windows_with_regime(df):
    """train_wind_estimator.build_windows()와 동일한 윈도잉이되, regime/
    t_since_transition_s도 같이 뽑아냄(둘 다 window의 '마지막 시점' 기준)."""
    X_list, y_list, regime_list, t_since_list, cond_list, t_abs_list = [], [], [], [], [], []

    for cond_idx, g in df.groupby("condition_idx"):
        g = g.sort_values("t_s").reset_index(drop=True)
        (g["roll_cos_yaw"], g["roll_sin_yaw"],
         g["pitch_cos_yaw"], g["pitch_sin_yaw"]) = yaw_decompose(
            g["roll_deg"].to_numpy(), g["pitch_deg"].to_numpy(), g["yaw_deg"].to_numpy())

        feats = g[FEATURES].to_numpy()
        wind_vx = g["wind_vx_m_s"].to_numpy()
        wind_vy = g["wind_vy_m_s"].to_numpy()
        regime = g["regime"].to_numpy()
        t_since = g["t_since_transition_s"].to_numpy()
        t_abs = g["t_s"].to_numpy()

        n = len(g)
        for i in range(WINDOW, n):
            X_list.append(feats[i - WINDOW:i].flatten())
            y_list.append([wind_vx[i], wind_vy[i]])
            regime_list.append(regime[i])
            t_since_list.append(t_since[i])
            t_abs_list.append(t_abs[i])
            cond_list.append(cond_idx)

    return (np.array(X_list, dtype=np.float32), np.array(y_list, dtype=np.float32),
            np.array(regime_list), np.array(t_since_list, dtype=np.float32),
            np.array(t_abs_list, dtype=np.float32), np.array(cond_list))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("--model", default="wind_estimator.pt")
    parser.add_argument("--bin-width", type=float, default=0.5,
                         help="t_since_transition_s를 몇 초 단위로 묶어서 볼지")
    args = parser.parse_args()

    ckpt = torch.load(args.model, map_location="cpu", weights_only=False)
    print(f"체크포인트: {args.model} (best_val_mae={ckpt['best_val_mae']:.3f}m/s)")
    model = WindPINN(ckpt["window"], len(ckpt["features"]))
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    X_mean = torch.tensor(ckpt["X_mean"], dtype=torch.float32)
    X_std = torch.tensor(ckpt["X_std"], dtype=torch.float32)

    df = pd.read_csv(args.csv)
    print(f"데이터: {args.csv}  행 수={len(df)}  에피소드 수={df['condition_idx'].nunique()}")

    X, y, regime, t_since, t_abs, cond = build_windows_with_regime(df)
    print(f"윈도우 샘플 수: {len(X)}")

    Xn = (torch.from_numpy(X) - X_mean) / X_std
    with torch.no_grad():
        pred = model(Xn)[:, :2].numpy()
    speed_err = np.abs(np.linalg.norm(pred, axis=1) - np.linalg.norm(y, axis=1))
    vec_err = np.linalg.norm(pred - y, axis=1)

    # --- 베이스라인: "정상상태"(전환 후 SEGMENT_DURATION 절반 이상 지난 시점) 오차 ---
    steady_mask = t_since >= 4.0  # 구간이 8초니까 절반 이상 지나면 '충분히 안정' 취급
    print(f"\n=== 베이스라인 (전환 후 4초 이상 지난 '정상상태' 구간, n={steady_mask.sum()}) ===")
    for reg in ["fixed", "gust"]:
        m = steady_mask & (regime == reg)
        if m.sum() > 0:
            print(f"  {reg:6s}  speed MAE={speed_err[m].mean():.3f}m/s  "
                  f"vec MAE={vec_err[m].mean():.3f}m/s  (n={m.sum()})")

    # --- t_since_transition_s 구간별 오차 (전환 직후 vs 시간 지남에 따라) ---
    print(f"\n=== 전환 후 경과시간별 오차 ({args.bin_width}s 단위 구간) ===")
    max_t = float(t_since.max())
    bins = np.arange(0.0, max_t + args.bin_width, args.bin_width)
    print(f"{'구간(s)':>12s}  {'전체 n':>8s}  {'speed MAE':>10s}  {'vec MAE':>10s}  "
          f"{'fixed로 전환':>14s}  {'gust로 전환':>13s}")
    for b0, b1 in zip(bins[:-1], bins[1:]):
        m = (t_since >= b0) & (t_since < b1)
        if m.sum() == 0:
            continue
        m_fixed = m & (regime == "fixed")
        m_gust = m & (regime == "gust")
        fixed_mae = f"{speed_err[m_fixed].mean():.3f} (n={m_fixed.sum()})" if m_fixed.sum() else "-"
        gust_mae = f"{speed_err[m_gust].mean():.3f} (n={m_gust.sum()})" if m_gust.sum() else "-"
        print(f"  {b0:5.1f}~{b1:<5.1f}  {m.sum():8d}  {speed_err[m].mean():10.3f}  "
              f"{vec_err[m].mean():10.3f}  {fixed_mae:>14s}  {gust_mae:>13s}")

    # --- 첫 구간(에피소드 시작, t_since==t_abs인 구간) 제외하고 "진짜 전환"만 다시 ---
    is_first_segment = np.isclose(t_since, t_abs)
    real_transition = ~is_first_segment
    print(f"\n=== 참고: 에피소드 첫 구간(전환이 아니라 그냥 시작) 제외 시 (n={real_transition.sum()}) ===")
    m0 = real_transition & (t_since < 0.5)
    m_late = real_transition & (t_since >= 4.0)
    if m0.sum() and m_late.sum():
        print(f"  전환 직후(0~0.5s): speed MAE={speed_err[m0].mean():.3f}m/s (n={m0.sum()})")
        print(f"  정상상태(4s+):     speed MAE={speed_err[m_late].mean():.3f}m/s (n={m_late.sum()})")
        print(f"  차이: {speed_err[m0].mean() - speed_err[m_late].mean():+.3f}m/s")


if __name__ == "__main__":
    main()
