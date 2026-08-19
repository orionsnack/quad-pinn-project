"""
repeat_sweep.sh가 여러 번 반복 실행한 pinn_correction_*.csv들(매니페스트로 나열)을
모아서 조건별 개선율(%)의 평균±표준편차를 낸다.

지금까지 이 프로젝트의 모든 A/B 결과는 "1회 측정"이었음 (EXPERIMENTS.md 12-21절
코드 리뷰 4번 항목) - 이 스크립트가 그 신뢰구간을 실제로 채워준다.

세 가지 CSV 스키마를 자동 인식:
  - pos_error_m 컬럼이 있으면 그걸 오차로 씀 (고정바람/gust 스윕)
  - 없으면 roll_deg/pitch_deg로 att_error = sqrt(roll^2+pitch^2) 계산 (회전 피드포워드)
phase 값은 "*_off"/"*_on" 접미사로 통일해서 처리.

사용법: python aggregate_repeats.py <manifest.txt 또는 csv 경로들...>
"""
import sys
import math
import csv
from collections import defaultdict


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def peak_errors_by_condition(rows):
    """조건(wind_label)별 {"off": peak, "on": peak} 반환."""
    has_pos_err = "pos_error_m" in rows[0] if rows else False
    peaks = defaultdict(lambda: {"off": 0.0, "on": 0.0})
    for r in rows:
        label = r["wind_label"]
        phase = r["phase"]
        key = "off" if phase.endswith("off") else ("on" if phase.endswith("on") else None)
        if key is None:
            continue
        if has_pos_err:
            err = float(r["pos_error_m"])
        else:
            roll, pitch = float(r["roll_deg"]), float(r["pitch_deg"])
            err = math.sqrt(roll ** 2 + pitch ** 2)
        peaks[label][key] = max(peaks[label][key], err)
    return peaks


def main():
    args = sys.argv[1:]
    if not args:
        print("사용법: python aggregate_repeats.py <manifest.txt 또는 csv 경로들...>")
        sys.exit(1)

    csv_paths = []
    if len(args) == 1 and args[0].endswith(".txt"):
        with open(args[0]) as f:
            csv_paths = [line.strip() for line in f if line.strip()]
    else:
        csv_paths = args

    if not csv_paths:
        print("집계할 CSV가 없음 (매니페스트가 비어있거나 전부 실패)")
        sys.exit(1)

    # condition -> list of improvement% (한 실행당 하나씩)
    improvements = defaultdict(list)
    off_vals = defaultdict(list)
    on_vals = defaultdict(list)

    for p in csv_paths:
        try:
            rows = load_rows(p)
        except FileNotFoundError:
            print(f"  [경고] 파일 없음, 스킵: {p}")
            continue
        if not rows:
            continue
        peaks = peak_errors_by_condition(rows)
        for label, v in peaks.items():
            off, on = v["off"], v["on"]
            if off <= 1e-6:
                continue
            improvements[label].append((off - on) / off * 100.0)
            off_vals[label].append(off)
            on_vals[label].append(on)

    n_runs = len(csv_paths)
    print(f"\n=== {n_runs}회 반복 집계 (조건 순서는 첫 실행 기준) ===\n")
    header = f"{'조건':<16} {'N':>3} {'OFF 평균':>10} {'ON 평균':>10} {'개선율 평균':>12} {'표준편차':>10} {'범위':>20}"
    print(header)
    print("-" * len(header))

    summary_rows = []
    for label, vals in improvements.items():
        n = len(vals)
        mean = sum(vals) / n
        var = sum((x - mean) ** 2 for x in vals) / n if n > 1 else 0.0
        std = math.sqrt(var)
        off_mean = sum(off_vals[label]) / n
        on_mean = sum(on_vals[label]) / n
        rng = f"{min(vals):+.1f}%~{max(vals):+.1f}%"
        print(f"{label:<16} {n:>3} {off_mean:>9.3f}m {on_mean:>9.3f}m {mean:>+11.1f}% {std:>9.1f}% {rng:>20}")
        summary_rows.append((label, n, off_mean, on_mean, mean, std, min(vals), max(vals)))

    out_path = csv_paths[0].rsplit("/", 1)[0] + "/aggregate_summary_" + \
        csv_paths[0].rsplit("/", 1)[-1].replace(".csv", "") + ".csv"
    try:
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["condition", "n_runs", "off_mean_m", "on_mean_m",
                        "improvement_mean_pct", "improvement_std_pct", "improvement_min_pct", "improvement_max_pct"])
            for row in summary_rows:
                w.writerow(row)
        print(f"\n요약 CSV 저장: {out_path}")
    except Exception as e:
        print(f"\n[경고] 요약 CSV 저장 실패: {e}")


if __name__ == "__main__":
    main()
