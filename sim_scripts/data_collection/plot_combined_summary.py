"""
세션 여러 개(같은 오케스트레이션 실행 안에서 만들어진 것들)를 합쳐서 보는 2개의
요약 PNG를 만듦:
  1) all_sessions_montage.png — 세션별 3x4 격자 PNG(plot_session_grid.py 산출물)들을
     5x3 모자이크 하나로 축소해서 붙임 (전체 흐름/밀도를 한눈에)
  2) all_trials_overlay.png — 모든 세션의 모든 조건(위치오차 시계열)을 원본 CSV에서
     직접 읽어 선 하나의 그래프에 다 겹침 (색=풍속, 투명도 낮춰서 밀도로 보이게)

run_yaw_collection_sessions.sh가 세션 전부 끝난 뒤 자동으로 호출하지만, 특정 PNG/CSV
묶음을 다시 합치고 싶을 때 수동으로도 씀:

  python plot_combined_summary.py --pngs figures/RUN/session_*.png \
      --csvs logs/wind_random_*_yaw*.csv --out-dir figures/RUN
"""

import argparse
import os
import re

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from PIL import Image, ImageDraw, ImageFont

_KO_FONT_CANDIDATES = [
    "/mnt/c/Windows/Fonts/NotoSansKR-VF.ttf",
    "/mnt/c/Windows/Fonts/malgun.ttf",
]


def _find_font():
    for path in _KO_FONT_CANDIDATES:
        if os.path.exists(path):
            return path
    return None


def _setup_mpl_font():
    path = _find_font()
    if path is None:
        print("  [경고] 한글 폰트를 못 찾음 - 기본 폰트로 진행")
        return matplotlib.rcParams["font.family"][0]
    fm.fontManager.addfont(path)
    return fm.FontProperties(fname=path).get_name()


def make_montage(png_paths, out_path, cols=5):
    font_path = _find_font()
    n = len(png_paths)
    rows = -(-n // cols)  # ceil

    thumb_w = 640
    label_h = 44
    pad = 10
    title_h = 90

    with Image.open(png_paths[0]) as im0:
        ratio = im0.height / im0.width
    thumb_h = int(thumb_w * ratio)

    cell_w = thumb_w + pad * 2
    cell_h = thumb_h + label_h + pad * 2
    canvas = Image.new("RGB", (cell_w * cols, title_h + cell_h * rows), "#f9f9f7")
    draw = ImageDraw.Draw(canvas)
    title_font = ImageFont.truetype(font_path, 30) if font_path else ImageFont.load_default()
    label_font = ImageFont.truetype(font_path, 22) if font_path else ImageFont.load_default()

    title = f"세션 {n}개 전체 모자이크"
    tw = draw.textlength(title, font=title_font)
    draw.text(((canvas.width - tw) / 2, 28), title, fill="#0b0b0b", font=title_font)

    for idx, fp in enumerate(png_paths):
        row, col = divmod(idx, cols)
        x0 = col * cell_w + pad
        y0 = title_h + row * cell_h + pad
        with Image.open(fp) as im:
            thumb = im.convert("RGB").resize((thumb_w, thumb_h), Image.LANCZOS)
        canvas.paste(thumb, (x0, y0))

        label = os.path.splitext(os.path.basename(fp))[0]
        label = re.sub(r"^session_", "", label)
        lw = draw.textlength(label, font=label_font)
        draw.text((x0 + max(0, (thumb_w - lw) / 2), y0 + thumb_h + 6), label,
                   fill="#52514e", font=label_font)

    canvas.save(out_path, optimize=True)
    print(f"저장됨: {out_path} ({canvas.size[0]}x{canvas.size[1]})")


def make_overlay(csv_paths, out_path):
    font_name = _setup_mpl_font()
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
        "axes.labelsize": 16,
        "xtick.labelsize": 13,
        "ytick.labelsize": 13,
    })

    segments, colors = [], []
    for fp in csv_paths:
        df = pd.read_csv(fp)
        df["pos_err"] = np.hypot(df.cmd_north_m - df.actual_north_m, df.cmd_east_m - df.actual_east_m)
        df["speed"] = np.hypot(df.wind_vx_m_s, df.wind_vy_m_s)
        for _, g in df.groupby("condition_idx"):
            g = g.sort_values("t_s")
            segments.append(np.column_stack([g.t_s.to_numpy(), g.pos_err.to_numpy()]))
            colors.append(g.speed.iloc[0])

    n_trials = len(segments)
    print(f"  총 트라이얼(선) 수: {n_trials}")
    speed_arr = np.array(colors)
    norm = Normalize(vmin=speed_arr.min(), vmax=speed_arr.max())
    cmap = plt.get_cmap("Blues")

    fig, ax = plt.subplots(figsize=(22, 13))
    lc = LineCollection(segments, colors=cmap(norm(speed_arr)), linewidths=0.6, alpha=0.10)
    ax.add_collection(lc)
    x_max = max(seg[:, 0].max() for seg in segments)
    y_max = np.quantile(np.concatenate([seg[:, 1] for seg in segments]), 0.999)
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("위치오차 (m)")
    ax.grid(True, color="#e1e0d9", linewidth=0.7)
    ax.spines[["top", "right"]].set_visible(False)

    ax.set_title(f"전체 {n_trials}개 트라이얼 위치오차 시계열 한 장에 겹치기", fontsize=24, pad=40)
    fig.text(0.5, 0.935,
              "선 하나 = 조건 1개 (t=0에 바람 온셋) · 색 = 풍속(진할수록 강함) · 선 투명도 낮춰서 밀도로 보이게 함",
              ha="center", fontsize=15, color="#52514e")

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.015)
    cbar.set_label("풍속 (m/s)", fontsize=15, color="#52514e")
    cbar.ax.tick_params(labelsize=13)

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(out_path, dpi=200, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    print(f"저장됨: {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pngs", nargs="*", default=[], help="세션별 grid PNG 목록 (모자이크용)")
    parser.add_argument("--csvs", nargs="*", default=[], help="세션별 CSV 목록 (전체 겹침 그래프용)")
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    if len(args.pngs) >= 2:
        make_montage(sorted(args.pngs), os.path.join(args.out_dir, "all_sessions_montage.png"))
    else:
        print("  [건너뜀] 모자이크: PNG가 2개 미만")

    if len(args.csvs) >= 1:
        make_overlay(sorted(args.csvs), os.path.join(args.out_dir, "all_trials_overlay.png"))
    else:
        print("  [건너뜀] 겹침 그래프: CSV 없음")
