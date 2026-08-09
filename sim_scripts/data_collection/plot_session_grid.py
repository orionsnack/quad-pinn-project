"""
세션 CSV 하나(yaw 12개 x 조건당 15개)를 3x4 격자 PNG로 그림.
run_yaw_collection_sessions.sh가 세션 하나 끝날 때마다 자동으로 이 스크립트를 호출함
(--out-dir에 저장). 수동으로 특정 CSV 하나만 다시 그리고 싶을 때도 그대로 씀:

  python plot_session_grid.py ../../logs/wind_random_TIMESTAMP_yaw12p0_n12x15_seed7042.csv

한글 폰트가 리눅스 쪽엔 없어서(WSL), Windows 쪽 Noto Sans KR을 직접 지정해서 씀 -
그 폰트 파일이 없는 환경(WSL이 아니거나 경로가 다르면)에서는 matplotlib 기본 폰트로
자동 폴백함(한글이 깨져 보일 수 있지만 스크립트 자체는 안 죽음).
"""

import argparse
import os
import sys

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

_KO_FONT_CANDIDATES = [
    "/mnt/c/Windows/Fonts/NotoSansKR-VF.ttf",
    "/mnt/c/Windows/Fonts/malgun.ttf",
]


def _setup_font():
    for path in _KO_FONT_CANDIDATES:
        if os.path.exists(path):
            fm.fontManager.addfont(path)
            return fm.FontProperties(fname=path).get_name()
    print("  [경고] 한글 폰트를 못 찾음 - 기본 폰트로 진행 (한글이 깨져 보일 수 있음)",
          file=sys.stderr)
    return matplotlib.rcParams["font.family"][0]


def make_grid(csv_path, out_path):
    font_name = _setup_font()
    plt.rcParams.update({
        "font.family": font_name,
        "axes.unicode_minus": False,
        "axes.edgecolor": "#c3c2b7",
        "axes.labelcolor": "#52514e",
        "xtick.color": "#898781",
        "ytick.color": "#898781",
        "axes.titlecolor": "#0b0b0b",
        "figure.facecolor": "#fcfcfb",
        "axes.facecolor": "#fcfcfb",
    })

    df = pd.read_csv(csv_path)
    df["pos_err"] = np.hypot(df.cmd_north_m - df.actual_north_m, df.cmd_east_m - df.actual_east_m)
    df["speed"] = np.hypot(df.wind_vx_m_s, df.wind_vy_m_s)

    n_total = df.condition_idx.nunique()
    if n_total % 12 != 0:
        print(f"  [경고] 조건 수({n_total})가 12로 안 나눠떨어짐 - "
              f"n-yaw=12 세션이 아닌 것 같음. 그래도 시도함.", file=sys.stderr)
    per_yaw = n_total // 12 if n_total % 12 == 0 else max(1, n_total // 12)
    df["yaw_group"] = df.condition_idx // per_yaw

    speed_min, speed_max = df.speed.min(), df.speed.max()
    norm = Normalize(vmin=speed_min, vmax=speed_max)
    cmap = plt.get_cmap("Blues")

    groups = sorted(df.yaw_group.unique())[:12]
    fig, axes = plt.subplots(3, 4, figsize=(17, 11), sharex=True, sharey=True)
    y_max = df.pos_err.quantile(0.995)

    for gid, ax in zip(groups, axes.flat):
        g = df[df.yaw_group == gid]
        yaw_rad = np.radians(g.yaw_deg)
        yaw_label = np.degrees(np.arctan2(np.sin(yaw_rad).mean(), np.cos(yaw_rad).mean()))
        for cond_idx, gg in g.groupby("condition_idx"):
            gg = gg.sort_values("t_s")
            ax.plot(gg.t_s, gg.pos_err, color=cmap(norm(gg.speed.iloc[0])),
                    linewidth=1.1, alpha=0.85, solid_capstyle="round")
        ax.set_title(f"yaw ≈ {yaw_label:.0f}°", fontsize=11, pad=6)
        ax.set_ylim(0, y_max * 1.08)
        ax.grid(True, color="#e1e0d9", linewidth=0.7)
        ax.spines[["top", "right"]].set_visible(False)

    for ax in axes[-1, :]:
        ax.set_xlabel("t (s)", fontsize=9)
    for ax in axes[:, 0]:
        ax.set_ylabel("위치오차 (m)", fontsize=9)

    base = os.path.basename(csv_path)
    fig.suptitle(f"{base} — 12개 yaw × 조건당 {per_yaw}개 위치오차 시계열", fontsize=13, y=0.995)
    fig.text(0.5, 0.965,
              "선 하나 = 무작위 바람 조건 1개 (t=0에 바람 온셋) · 색 = 풍속(진할수록 강함)",
              ha="center", fontsize=10, color="#52514e")

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    cbar_ax = fig.add_axes([0.93, 0.15, 0.012, 0.7])
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label("풍속 (m/s)", fontsize=9, color="#52514e")
    cbar.ax.tick_params(labelsize=8)

    fig.tight_layout(rect=[0, 0, 0.92, 0.94])
    fig.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--out-dir", default="../../figures")
    parser.add_argument("--out-name", default=None,
                         help="생략하면 CSV 파일명에서 자동으로 만듦 (session_<csv이름>.png)")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    out_name = args.out_name or ("session_" + os.path.splitext(os.path.basename(args.csv_path))[0] + ".png")
    out_path = os.path.join(args.out_dir, out_name)

    make_grid(args.csv_path, out_path)
    print(f"저장됨: {out_path}")
