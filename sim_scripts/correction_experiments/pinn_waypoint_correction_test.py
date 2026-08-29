"""
지금까지의 A/B(pinn_wind_correction_sweep.py 등)는 전부 "제자리를 유지하는 동안
바람에 얼마나 안 밀리는지"만 쟀음 - 목표 위치가 한 번도 안 바뀌었음. 이 스크립트는
반대로 "임의의 목표 좌표를 찍으면, 바람이 부는 중에도 PINN 보정을 켠 채로 그
지점까지 정확하게 도달하는지"를 검증한다(2026-08-24, 사용자 요청으로 프로젝트
목표가 "실기 실측+시뮬 결합"에서 "SITL만으로 웨이포인트 이동 정확도 검증"으로
재조정됨 - 실기 부분은 스코프에서 제외).

트라이얼 구조(조건마다):
  홈 위치로 복귀+안정화(무풍) -> 바람 온셋 -> 목표 웨이포인트(홈에서 동쪽으로
  WAYPOINT_OFFSET_EAST_M) 커맨드 -> 보정 OFF로 TRIAL_DURATION_S초 관찰 ->
  같은 과정을 보정 ON으로 반복
지표(2026-08-28 재설계, EXPERIMENTS.md 12-47절): 처음엔 "정상상태 위치오차"
(트라이얼 마지막 SETTLE_WINDOW_S초 동안의 pos_error 평균)를 썼는데, 실측해보니
PX4 PID의 적분 항이 도달 후 몇 초 안에 바람을 거의 다 상쇄해버려서 그 시점 이후를
재면 보정 효과가 안 보임(strong 조건 실측: 도달 직후 피크 0.52m -> 4초 만에
0.15m -> 7초 뒤엔 노이즈 바닥 0.02~0.03m). 대신 **"목표에 처음 근접한 직후,
아직 적분이 다 안 감긴 구간의 피크 오차"**(arrival_peak_error)를 주 지표로 씀 -
기존 실험들(3.1/3.3절)이 "바람 온셋 직후 피크"를 쓴 것과 같은 철학. 정상상태
오차도 참고용으로 계속 같이 출력함.

바람 조건은 pinn_wind_correction_sweep.py와 동일한 5개 벡터를 그대로 재사용
(ACCEL_GAIN=0.05 등 기존 최선 파라미터와 직접 비교 가능하도록). 이동 축을
"동쪽"으로 잡았기 때문에 "crosswind"(vx=0,vy=6, 북쪽 성분만)는 이 실험에서
말 그대로 진행방향과 수직인 옆바람이 되고, 나머지(light/default/strong)는
동쪽 성분이 있어 순풍 성분이 섞여 있음 - 실제 웨이포인트 비행에서 바람이
진행방향과 반드시 수직일 필요는 없으므로 이 자체는 문제 아님.

실행 전 조건: WSL에서 PX4 SITL이 windy 월드로 돌고 있어야 함
(HEADLESS=1 make px4_sitl gz_x500_windy)
"""

import argparse
import asyncio
import csv
import datetime
import functools
import sys
import time
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw, AccelerationNed

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "offline_training"))
from wind_pinn_model import FEATURES, yaw_decompose  # noqa: E402
from correction_common import (  # noqa: E402
    set_wind as _set_wind, PINNCorrector, EmaSmoother, apply_deadband,
    connect_and_wait_ready, arm_and_takeoff, start_telemetry_monitors, start_offboard_hold,
)


# ============================================================
# 실험 파라미터
# ============================================================
WORLD_NAME = "windy"
set_wind = functools.partial(_set_wind, WORLD_NAME)
WIND_CONDITIONS = [
    ("calm", 0.0, 0.0),
    ("light", 2.0, 1.0),
    ("default", 5.0, 2.0),
    ("strong", 8.0, 3.0),
    ("crosswind", 0.0, 6.0),
]
ACCEL_GAIN = 0.05        # 12-31절 재스윕 확정값 그대로 재사용 (적응형 게인/더더링은
MAX_ACCEL_MPS2 = 2.0     # 이 첫 버전에서는 단순화를 위해 뺌 - 필요하면 나중에 추가)
WIND_DEADBAND_MPS = 1.0

WAYPOINT_OFFSET_NORTH_M = 0.0
WAYPOINT_OFFSET_EAST_M = 10.0   # 홈에서 동쪽으로 10m 지점을 목표로 이동

TRIAL_DURATION_S = 15.0
# 2026-08-28 지표 재설계: 처음엔 "마지막 5초 평균"(정상상태 오차)을 썼는데,
# 실측해보니 PX4 PID의 적분 항이 도달 후 ~4초 안에 바람을 거의 다 상쇄해버려서
# (strong 조건 실측: 도달 직후 피크 0.52m -> 4초 만에 0.15m -> 7초 뒤엔 0.02~0.03m
# 로 노이즈 바닥) 그 이후 구간을 재면 보정 효과가 안 보임. 대신 "목표에 가장
# 근접한 직후, 아직 적분이 다 안 감긴 구간"의 피크 오차를 잼 - 기존 실험들(3.1/
# 3.3절)이 "바람 온셋 직후 피크"를 쓴 것과 같은 철학.
#
# 1차 시도(문턱 기반, 실패): "ARRIVAL_THRESHOLD_M 이내로 처음 들어온 시점"을
# 기준으로 삼았더니, 아직 감속 중이라 그 문턱값 자체(~2m)가 항상 window의
# 최댓값이 돼버려 5조건 전부 1.7~2.0m로 나옴(2026-08-28 밤 발견, 미수정 상태로
# 커밋됨). "최근접 시점"(pos_error 최솟값 순간, 이미 감속 끝난 뒤) 기준으로
# 교체 - SITL 재검증(2026-08-29)해보니 또 다른 버그 발견: "최근접 시점"을
# 트라이얼 전체(15초)에서 찾다 보니, 실제 도달(t≈2.3~2.9초)보다 한참 뒤인
# t≈8~13초 사이에서 PX4의 자연스러운 미세 진동(노이즈 바닥, ~0.02~0.07m 진폭 -
# 12-47절에서도 이미 확인된 현상)이 우연히 더 작은 값을 찍는 순간이 있고, 그게
# "최근접"으로 잘못 뽑혀 window가 진짜 도달 직후가 아니라 이미 다 안정화된 뒤의
# 노이즈 구간을 보고 있었음(strong 조건 실측: 진짜 도달 피크는 t=2.90에 0.509m
# 였는데, 지표는 t=8.20~13.20 구간의 노이즈 피크 0.068m를 보고함). 최근접 탐색을
# `EARLY_SEARCH_WINDOW_S`(실측 transit 시간 2.3~2.9초 기준으로 넉넉하게 6초)
# 이내로 제한해서 고침 - 아직 이 수정판은 SITL로 재검증 안 됨.
EARLY_SEARCH_WINDOW_S = 6.0  # 이 시간 안에서만 "최근접 시점"을 찾음(그 이후의
                              # 노이즈 바닥 진동에 낚이지 않도록)
ARRIVAL_WINDOW_S = 5.0      # 최근접 시점 이후 이 기간 동안의 peak pos_error가 지표
SETTLE_WINDOW_S = 5.0     # 마지막 5초 동안의 pos_error 평균 = "정상상태 위치오차"
                          # (참고용으로 계속 계산·출력함 - 적분이 다 감긴 뒤 상태 확인용)
RETURN_HOME_WAIT_S = 10.0  # 조건 전환마다 홈으로 복귀+안정화하는 대기 시간 -
                            # 실제로 ~10m를 왕복 비행해야 하므로 기존 PHASE_GAP_S(4초,
                            # 제자리 유지 실험 기준)보다 넉넉하게 잡음
SAFE_ALTITUDE_M = 1.5
TAKEOFF_TIMEOUT_S = 15.0
LOG_INTERVAL_S = 0.05
SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

MODEL_PATH = Path(__file__).parent.parent.parent / "offline_training" / "wind_estimator.pt"


async def run():
    drone = System()

    await connect_and_wait_ready(drone)

    print(f"\n모델 로드: {MODEL_PATH}")
    corrector = PINNCorrector(MODEL_PATH)
    print(f"  window={corrector.window}  features={corrector.features}")
    print(f"  게인 모드: 고정 (ACCEL_GAIN={ACCEL_GAIN})")
    print(f"  목표 웨이포인트: 홈에서 (north={WAYPOINT_OFFSET_NORTH_M}, east={WAYPOINT_OFFSET_EAST_M})m")

    await set_wind(0.0, 0.0)

    if not await arm_and_takeoff(drone, SAFE_ALTITUDE_M, TAKEOFF_TIMEOUT_S):
        return

    latest_pv, latest_att, latest_gyro, pv_task, att_task, gyro_task = \
        await start_telemetry_monitors(drone)

    nominal = {"north": latest_pv["north"], "east": latest_pv["east"],
               "down": latest_pv["down"], "yaw": latest_att["yaw"]}
    current_cmd = {"accel_n": 0.0, "accel_e": 0.0}
    send_gaps = []

    async def offboard_sender():
        next_tick = time.monotonic()
        last_sent = None
        while True:
            try:
                await drone.offboard.set_position_velocity_acceleration_ned(
                    PositionNedYaw(nominal["north"], nominal["east"],
                                    nominal["down"], nominal["yaw"]),
                    VelocityNedYaw(0.0, 0.0, 0.0, nominal["yaw"]),
                    AccelerationNed(current_cmd["accel_n"], current_cmd["accel_e"], 0.0),
                )
            except Exception as exc:
                print(f"  [!!! offboard_sender 예외] {type(exc).__name__}: {exc}")
            now = time.monotonic()
            if last_sent is not None:
                send_gaps.append(now - last_sent)
            last_sent = now
            next_tick += SEND_PERIOD_S
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    print("\n--- Offboard 진입 준비: 초기 setpoint(현재 위치 고정) 전송 ---")
    if not await start_offboard_hold(drone, nominal):
        return

    sender_task = asyncio.create_task(offboard_sender())

    home = dict(nominal)
    target = {"north": home["north"] + WAYPOINT_OFFSET_NORTH_M,
              "east": home["east"] + WAYPOINT_OFFSET_EAST_M,
              "down": home["down"], "yaw": home["yaw"]}
    print(f"  홈=({home['north']:.2f},{home['east']:.2f})  "
          f"목표=({target['north']:.2f},{target['east']:.2f})")

    # 2026-08-29 추가: 홈->목표 직선에서 얼마나 옆으로 벗어났는지(경로 이탈량).
    # "도착직후 피크오차"(arrival_peak_error)와 달리 "언제 도착했는지"를 정의할
    # 필요가 없어 그 정의를 둘러싼 두 번의 버그(12-47/48)를 애초에 피해감 -
    # 트라이얼 시작(홈 근처, 직선 위)과 끝(목표 근처, 직선 위)에서 자연히 0에
    # 가까워지므로 트라이얼 전체에서의 peak를 그냥 쓰면 됨.
    home_to_target_n = target["north"] - home["north"]
    home_to_target_e = target["east"] - home["east"]
    home_to_target_dist = (home_to_target_n ** 2 + home_to_target_e ** 2) ** 0.5

    def cross_track_error(north, east):
        if home_to_target_dist < 1e-6:
            return ((north - home["north"]) ** 2 + (east - home["east"]) ** 2) ** 0.5
        # 2D 외적으로 부호 있는 수선 거리 계산, 크기만 씀
        v_n, v_e = north - home["north"], east - home["east"]
        return abs(v_e * home_to_target_n - v_n * home_to_target_e) / home_to_target_dist

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"../../logs/pinn_waypoint_correction_{timestamp_str}.csv"
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "wind_label", "phase", "t_s", "wind_est_north", "wind_est_east",
        "accel_n_m_s2", "accel_e_m_s2",
        "target_north_m", "target_east_m", "actual_north_m", "actual_east_m",
        "pos_error_m", "cross_track_error_m", "roll_deg", "pitch_deg",
    ])
    print(f"\nCSV 로그 저장 경로: {csv_path}")

    async def run_trial(wind_label, phase_name, use_correction):
        corrector.buffer.clear()
        smoother_n = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        smoother_e = EmaSmoother(time_constant_s=0.4, dt_s=LOG_INTERVAL_S)
        n_steps = int(TRIAL_DURATION_S / LOG_INTERVAL_S)
        settle_steps = int(SETTLE_WINDOW_S / LOG_INTERVAL_S)
        arrival_window_steps = int(ARRIVAL_WINDOW_S / LOG_INTERVAL_S)
        early_search_steps = int(EARLY_SEARCH_WINDOW_S / LOG_INTERVAL_S)
        next_log = time.monotonic()
        peak_error = 0.0
        settle_errors = []
        all_errors = []   # 트라이얼 전체 pos_error - 사후에 "최근접 시점" 찾는 데 씀
        peak_cross_track = 0.0

        for i in range(n_steps):
            t = i * LOG_INTERVAL_S
            north, east = latest_pv["north"], latest_pv["east"]
            vn, ve = latest_pv["vn"], latest_pv["ve"]
            roll, pitch, yaw = latest_att["roll"], latest_att["pitch"], latest_att["yaw"]

            r_cos, r_sin, p_cos, p_sin = yaw_decompose(roll, pitch, yaw)
            feat = {"vn_m_s": vn, "ve_m_s": ve, "roll_cos_yaw": r_cos, "roll_sin_yaw": r_sin,
                    "pitch_cos_yaw": p_cos, "pitch_sin_yaw": p_sin,
                    "wx_rad_s": latest_gyro["wx"], "wy_rad_s": latest_gyro["wy"],
                    "wz_rad_s": latest_gyro["wz"]}
            corrector.push_state([feat[f] for f in FEATURES])

            wind_n, wind_e = 0.0, 0.0
            if use_correction and corrector.ready():
                wind_enu = corrector.predict_raw()
                raw_n, raw_e = wind_enu[1].item(), wind_enu[0].item()
                wind_n = smoother_n.update(raw_n)
                wind_e = smoother_e.update(raw_e)
                wind_n, wind_e = apply_deadband(wind_n, wind_e, WIND_DEADBAND_MPS)
                accel_n = max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, -ACCEL_GAIN * wind_n))
                accel_e = max(-MAX_ACCEL_MPS2, min(MAX_ACCEL_MPS2, -ACCEL_GAIN * wind_e))
                current_cmd["accel_n"] = accel_n
                current_cmd["accel_e"] = accel_e
            else:
                current_cmd["accel_n"] = 0.0
                current_cmd["accel_e"] = 0.0

            pos_error = ((north - nominal["north"]) ** 2 + (east - nominal["east"]) ** 2) ** 0.5
            peak_error = max(peak_error, pos_error)
            if i >= n_steps - settle_steps:
                settle_errors.append(pos_error)
            all_errors.append(pos_error)

            xtrack = cross_track_error(north, east)
            peak_cross_track = max(peak_cross_track, xtrack)

            writer.writerow([
                wind_label, phase_name, f"{t:.2f}", f"{wind_n:.2f}", f"{wind_e:.2f}",
                f"{current_cmd['accel_n']:.3f}", f"{current_cmd['accel_e']:.3f}",
                f"{nominal['north']:.3f}", f"{nominal['east']:.3f}",
                f"{north:.3f}", f"{east:.3f}", f"{pos_error:.3f}", f"{xtrack:.3f}",
                f"{roll:.2f}", f"{pitch:.2f}",
            ])

            next_log += LOG_INTERVAL_S
            sleep_time = next_log - time.monotonic()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                next_log = time.monotonic()

        steady_state_error = sum(settle_errors) / len(settle_errors) if settle_errors else peak_error
        # "도달 시점"을 ARRIVAL_THRESHOLD_M 문턱 최초 통과로 정의했더니, 접근
        # 중이라 아직 감속 못 한 그 문턱값 자체(~2m)가 매번 창(window)의 최댓값이
        # 돼버려 바람에 의한 되돌아옴(실측 0.52m)보다 훨씬 크게 잡히는 버그가 있었음
        # (2026-08-28 밤 5조건 스모크테스트에서 전부 1.7~2.0m로 나와 발견 -
        # EXPERIMENTS.md 12-47절 참고). "최근접 시점" 기준으로 바꿨는데, 이번엔
        # 그 탐색을 트라이얼 전체(15초)에서 하다 보니 실제 도달(2.3~2.9초)보다
        # 한참 뒤 노이즈 바닥 진동의 우연한 저점을 "최근접"으로 잘못 집는 버그가
        # 또 나옴(2026-08-29 재검증에서 발견 - EXPERIMENTS.md 12-48/49절 참고).
        # EARLY_SEARCH_WINDOW_S 이내로만 탐색을 제한해 진짜 도달 시점만 잡히게 함.
        early_slice = all_errors[:early_search_steps] if all_errors else [peak_error]
        closest_idx = min(range(len(early_slice)), key=lambda k: early_slice[k])
        arrival_window_slice = all_errors[closest_idx: closest_idx + arrival_window_steps]
        arrival_peak_error = max(arrival_window_slice) if arrival_window_slice else peak_error
        return arrival_peak_error, steady_state_error, peak_error, peak_cross_track

    async def return_home():
        await set_wind(0.0, 0.0)
        current_cmd["accel_n"] = 0.0
        current_cmd["accel_e"] = 0.0
        nominal["north"] = home["north"]
        nominal["east"] = home["east"]
        await asyncio.sleep(RETURN_HOME_WAIT_S)

    results = []
    for wind_label, wind_vx, wind_vy in WIND_CONDITIONS:
        print(f"\n{'='*60}\n=== 바람 조건: {wind_label} (vx={wind_vx}, vy={wind_vy}) ===\n{'='*60}")

        print("  -- 웨이포인트 이동 보정 OFF --")
        await return_home()
        await set_wind(wind_vx, wind_vy)
        nominal["north"] = target["north"]
        nominal["east"] = target["east"]
        arr_off, settle_off, peak_off, xtrack_off = \
            await run_trial(wind_label, "waypoint_off", use_correction=False)
        print(f"    도달직후 피크오차(OFF) = {arr_off:.3f}m  경로이탈peak = {xtrack_off:.3f}m  "
              f"(정상상태={settle_off:.3f}m, 이동중peak={peak_off:.3f}m)")

        print("  -- 웨이포인트 이동 보정 ON --")
        await return_home()
        await set_wind(wind_vx, wind_vy)
        nominal["north"] = target["north"]
        nominal["east"] = target["east"]
        arr_on, settle_on, peak_on, xtrack_on = \
            await run_trial(wind_label, "waypoint_on", use_correction=True)
        print(f"    도달직후 피크오차(ON)  = {arr_on:.3f}m  경로이탈peak = {xtrack_on:.3f}m  "
              f"(정상상태={settle_on:.3f}m, 이동중peak={peak_on:.3f}m)")

        improvement = (arr_off - arr_on) / arr_off * 100 if arr_off > 1e-6 else 0.0
        xtrack_improvement = (xtrack_off - xtrack_on) / xtrack_off * 100 if xtrack_off > 1e-6 else 0.0
        print(f"    개선율(도달직후) = {improvement:+.1f}%   개선율(경로이탈) = {xtrack_improvement:+.1f}%")
        results.append((wind_label, wind_vx, wind_vy, arr_off, arr_on, improvement,
                         xtrack_off, xtrack_on, xtrack_improvement))

    await return_home()
    csv_file.close()

    print(f"\n{'='*70}\n=== 전체 요약 ===\n{'='*70}")
    print(f"{'조건':10s} {'풍속':>6s} {'도달OFF':>8s} {'도달ON':>8s} {'개선율':>8s} "
          f"{'경로OFF':>8s} {'경로ON':>8s} {'개선율':>8s}")
    for label, vx, vy, off, on, imp, xoff, xon, ximp in results:
        speed = (vx**2 + vy**2) ** 0.5
        print(f"{label:10s} {speed:6.2f} {off:8.3f} {on:8.3f} {imp:+7.1f}% "
              f"{xoff:8.3f} {xon:8.3f} {ximp:+7.1f}%")

    n_improved = sum(1 for r in results if r[5] > 0)
    n_xtrack_improved = sum(1 for r in results if r[8] > 0)
    print(f"\n도달직후 개선된 조건: {n_improved}/{len(results)}   "
          f"경로이탈 개선된 조건: {n_xtrack_improved}/{len(results)}")
    print(f"CSV 저장됨: {csv_path}")

    if send_gaps:
        n_over = sum(1 for g in send_gaps if g > 1.5 * SEND_PERIOD_S)
        print(
            f"\n[송신 간격 통계] 목표={SEND_PERIOD_S*1000:.0f}ms  "
            f"평균={sum(send_gaps)/len(send_gaps)*1000:.1f}ms  "
            f"최대={max(send_gaps)*1000:.1f}ms  임계치 초과={n_over}/{len(send_gaps)}"
        )

    cleanup_tasks = (sender_task, pv_task, att_task, gyro_task)
    for task in cleanup_tasks:
        task.cancel()
    for task in cleanup_tasks:
        try:
            await task
        except asyncio.CancelledError:
            pass

    print("\n--- Offboard 종료 ---")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard 종료 실패: {error._result.result}")

    print("\n--- Landing ---")
    await drone.action.land()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-> 착륙 완료 및 디스암 확인")
            break

    print("\nPINN 웨이포인트 보정 테스트 완료.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", choices=[c[0] for c in WIND_CONDITIONS], default=None,
                         help="지정하면 그 조건 하나만 실행 (빠른 검증/반복측정용)")
    args = parser.parse_args()
    if args.condition:
        WIND_CONDITIONS = [c for c in WIND_CONDITIONS if c[0] == args.condition]
    asyncio.run(run())
