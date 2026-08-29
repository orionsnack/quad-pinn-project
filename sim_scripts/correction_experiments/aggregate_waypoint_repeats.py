"""
repeat_sweep.sh가 여러 번 반복 실행한 pinn_waypoint_correction_*.csv들(매니페스트로
나열)을 모아서 조건별 개선율(%)의 평균±표준편차를 낸다. aggregate_repeats.py의
웨이포인트 버전 - 그 스크립트를 그대로 못 쓰는 이유는, 그건 phase 전체의
"peak pos_error"를 그대로 쓰는데 웨이포인트 트라이얼은 시작부터 목표까지 ~10m를
실제로 이동하는 구간이 있어서 "전체 peak"가 그 초반 이동거리 자체가 돼버려
의미가 없기 때문(EXPERIMENTS.md 12-47절에서 겪은 문제).

이 스크립트는 `pinn_waypoint_correction_test.py`의 `run_trial()`과 동일한
두 지표를 CSV의 원시 시계열(t_s/pos_error_m/cross_track_error_m)에서 다시
계산한다:
  - arrival_peak_error: EARLY_SEARCH_WINDOW_S 이내에서 pos_error_m이 가장 작았던
    시점("최근접 시점", 두 번의 버그 끝에 확정된 정의 - 12-48/49절)부터
    ARRIVAL_WINDOW_S 동안의 peak pos_error_m. "도달 직후 바람에 얼마나 밀리는지".
  - peak_cross_track: 트라이얼 전체(phase 전체)에서 cross_track_error_m의 최댓값.
    "도착 시점" 정의가 필요 없어 그대로 phase 전체 peak를 쓰면 됨(12-50절).

두 상수(EARLY_SEARCH_WINDOW_S, ARRIVAL_WINDOW_S)는 `pinn_waypoint_correction_test.py`
와 반드시 같은 값을 유지해야 함 - 그 스크립트가 바뀌면 여기도 같이 바꿀 것.

사용법: python aggregate_waypoint_repeats.py <manifest.txt 또는 csv 경로들...>
"""
import sys
import csv
import math
from collections import defaultdict

EARLY_SEARCH_WINDOW_S = 6.0
ARRIVAL_WINDOW_S = 5.0
LOG_INTERVAL_S = 0.05   # pinn_waypoint_correction_test.py의 LOG_INTERVAL_S와 반드시 일치해야 함 -
                        # 원본은 이 값으로 만든 "스텝 인덱스" 슬라이스(all_errors[:N])를 쓰는데,
                        # 여기서 시간(t_s) 비교로 "<=" 냐 "<" 냐를 잘못 맞추면 경계 스텝 하나
                        # 차이로 다른 "최근접 시점"이 나올 수 있음(실제로 겪음 - crosswind
                        # 조건에서 t=5.95의 0.171m과 t=6.00의 0.165m 사이 경계) - 그래서
                        # 시간 비교 대신 원본과 똑같이 행(row) 개수 기준 슬라이스를 씀.


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def phase_key(phase):
    if phase.endswith("off"):
        return "off"
    if phase.endswith("on"):
        return "on"
    return None


def compute_trial_metrics(trial_rows):
    """한 트라이얼(한 wind_label, 한 phase)의 행들에서 arrival_peak_error와
    peak_cross_track을 계산 - pinn_waypoint_correction_test.py의 run_trial()과
    똑같이 "행 개수" 기준 슬라이스로 재현(시간값 비교 아님 - 경계 스텝에서
    둘이 어긋나는 걸 실제로 겪음, 위 LOG_INTERVAL_S 주석 참고)."""
    trial_rows = sorted(trial_rows, key=lambda r: float(r["t_s"]))
    pos_errors = [float(r["pos_error_m"]) for r in trial_rows]
    cross_tracks = [float(r["cross_track_error_m"]) for r in trial_rows]

    early_search_steps = int(EARLY_SEARCH_WINDOW_S / LOG_INTERVAL_S)
    arrival_window_steps = int(ARRIVAL_WINDOW_S / LOG_INTERVAL_S)

    early_slice = pos_errors[:early_search_steps] if pos_errors else [0.0]
    closest_idx = min(range(len(early_slice)), key=lambda k: early_slice[k])
    arrival_window_slice = pos_errors[closest_idx: closest_idx + arrival_window_steps]
    arrival_peak_error = max(arrival_window_slice) if arrival_window_slice else max(pos_errors)
    peak_cross_track = max(cross_tracks) if cross_tracks else 0.0
    return arrival_peak_error, peak_cross_track


def trial_metrics_by_condition(rows):
    """조건(wind_label)별 {"off": {...}, "on": {...}} 반환 - 각각
    {"arrival": arrival_peak_error, "xtrack": peak_cross_track}."""
    grouped = defaultdict(list)
    for r in rows:
        key = phase_key(r["phase"])
        if key is None:
            continue
        grouped[(r["wind_label"], key)].append(r)

    result = defaultdict(dict)
    for (label, key), trial_rows in grouped.items():
        arrival, xtrack = compute_trial_metrics(trial_rows)
        result[label][key] = {"arrival": arrival, "xtrack": xtrack}
    return result


def mean_std(vals):
    n = len(vals)
    mean = sum(vals) / n
    var = sum((x - mean) ** 2 for x in vals) / n if n > 1 else 0.0
    return mean, math.sqrt(var)


def main():
    args = sys.argv[1:]
    if not args:
        print("사용법: python aggregate_waypoint_repeats.py <manifest.txt 또는 csv 경로들...>")
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

    # condition -> metric -> list of (off, on, improvement%) 한 실행당 하나씩
    arrival_off = defaultdict(list)
    arrival_on = defaultdict(list)
    arrival_imp = defaultdict(list)
    xtrack_off = defaultdict(list)
    xtrack_on = defaultdict(list)
    xtrack_imp = defaultdict(list)

    for p in csv_paths:
        try:
            rows = load_rows(p)
        except FileNotFoundError:
            print(f"  [경고] 파일 없음, 스킵: {p}")
            continue
        if not rows:
            continue
        by_cond = trial_metrics_by_condition(rows)
        for label, phases in by_cond.items():
            if "off" not in phases or "on" not in phases:
                continue
            a_off, a_on = phases["off"]["arrival"], phases["on"]["arrival"]
            x_off, x_on = phases["off"]["xtrack"], phases["on"]["xtrack"]

            if a_off > 1e-6:
                arrival_off[label].append(a_off)
                arrival_on[label].append(a_on)
                arrival_imp[label].append((a_off - a_on) / a_off * 100.0)
            if x_off > 1e-6:
                xtrack_off[label].append(x_off)
                xtrack_on[label].append(x_on)
                xtrack_imp[label].append((x_off - x_on) / x_off * 100.0)

    n_runs = len(csv_paths)
    print(f"\n=== {n_runs}회 반복 집계 (조건 순서는 첫 실행 기준) ===")

    def print_table(title, off_d, on_d, imp_d):
        print(f"\n--- {title} ---")
        header = f"{'조건':<16} {'N':>3} {'OFF 평균':>10} {'ON 평균':>10} {'개선율 평균':>12} {'표준편차':>10} {'범위':>20}"
        print(header)
        print("-" * len(header))
        rows_out = []
        for label, vals in imp_d.items():
            n = len(vals)
            mean, std = mean_std(vals)
            off_mean, _ = mean_std(off_d[label])
            on_mean, _ = mean_std(on_d[label])
            rng = f"{min(vals):+.1f}%~{max(vals):+.1f}%"
            print(f"{label:<16} {n:>3} {off_mean:>9.3f}m {on_mean:>9.3f}m {mean:>+11.1f}% {std:>9.1f}% {rng:>20}")
            rows_out.append((label, n, off_mean, on_mean, mean, std, min(vals), max(vals)))
        return rows_out

    arrival_rows = print_table("도달직후 피크오차 (arrival_peak_error)", arrival_off, arrival_on, arrival_imp)
    xtrack_rows = print_table("경로 이탈량 (peak_cross_track)", xtrack_off, xtrack_on, xtrack_imp)

    out_path = csv_paths[0].rsplit("/", 1)[0] + "/aggregate_waypoint_summary_" + \
        csv_paths[0].rsplit("/", 1)[-1].replace(".csv", "") + ".csv"
    try:
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "condition", "n_runs", "off_mean_m", "on_mean_m",
                        "improvement_mean_pct", "improvement_std_pct",
                        "improvement_min_pct", "improvement_max_pct"])
            for row in arrival_rows:
                w.writerow(("arrival_peak_error",) + row)
            for row in xtrack_rows:
                w.writerow(("peak_cross_track",) + row)
        print(f"\n요약 CSV 저장: {out_path}")
    except Exception as e:
        print(f"\n[경고] 요약 CSV 저장 실패: {e}")


if __name__ == "__main__":
    main()
