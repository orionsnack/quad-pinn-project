# 파일 사용법 요약

각 파일이 뭘 하는 파일이고 어떻게 실행하는지 빠르게 찾기 위한 문서입니다. 프로젝트
개요/결과 요약은 [README.md](README.md), 배경/설계 이유(왜 이렇게 만들었는지, 실패한
시도, 실험 결과)는 [EXPERIMENTS.md](EXPERIMENTS.md), 환경 구축·트러블슈팅은
[setup_guide.md](setup_guide.md) 참고.

**공통 전제**
- 다른 터미널에서 PX4 SITL이 `pxh>` 상태로 떠 있어야 함
  (`cd ~/MyProjects/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500` 또는 `gz_x500_windy`)
- `sim_scripts/`의 스크립트는 `conda activate px4sim` 후 실행
- `offline_training/`의 학습 스크립트는 `conda activate pinn_train` 후 실행
- 실행 위치는 항상 해당 파일이 있는 디렉터리 (`sim_scripts/data_collection/`,
  `sim_scripts/correction_experiments/`, 또는 `offline_training/`)

---

## sim_scripts/data_collection/

PINN 학습용 데이터를 모으는 스크립트들.

| 파일 | 용도 | 필요 SITL 월드 | 실행 명령 | 결과 저장 위치 |
|---|---|---|---|---|
| `wind_random_sweep.py` | 고정바람, yaw 그리드(기본 24x10)로 라벨링. yaw도 그리드로 도는 이유: 12-10절 | `gz_x500_windy` 필수 | `python wind_random_sweep.py` | `../../logs/wind_random_TIMESTAMP.csv` |
| `wind_random_dither_sweep.py` | 위와 동일 + 데이터 수집 중 무작위 토크 주입(더더링) — 12-17절 이유 | `gz_x500_windy` 필수 | `python wind_random_dither_sweep.py` | `../../logs/wind_random_dither_TIMESTAMP.csv` |
| `wind_gust_sweep.py` | gust(사인파 변동바람), yaw 그리드(기본 12x5)로 라벨링. 자동으로 격자 PNG도 생성 | `gz_x500_windy` 필수 | `python wind_gust_sweep.py` | `../../logs/wind_gust_TIMESTAMP.csv` + PNG |
| `wind_transition_sweep.py` | 고정↔gust 전환 구간 조사용(연구용, 학습 파이프라인 편입 목적 아님) — 12-27/32절 | `gz_x500_windy` 필수 | `python wind_transition_sweep.py` | `../../logs/wind_transition_TIMESTAMP.csv` |
| `wind_yaw_generalization_test.py` | 학습 yaw 밖 소규모(기본 8개) 검증 데이터 — `evaluate_checkpoint.py`와 세트 | `gz_x500_windy` 필수 | `python wind_yaw_generalization_test.py --yaw 180 --n 10` | `../../logs/wind_yawtest_{yaw}deg_TIMESTAMP.csv` |
| `run_yaw_collection_sessions.sh` / `run_yaw_gust_collection_sessions.sh` | `wind_random_sweep.py`/`wind_gust_sweep.py`를 여러 세션으로 나눠 반복, 세션마다 SITL 재시작(장시간 수집을 안전하게) | 스크립트가 직접 SITL을 껐다 켬 | `./run_yaw_collection_sessions.sh --sessions 4 --n-yaw 24 --n-per-yaw 10` | 세션마다 별도 CSV + 세션/모자이크 PNG |
| `watch_collection.sh` | 위 오케스트레이션 스크립트가 멈췄는지 감시하는 워치독 | - | `./watch_collection.sh` | 콘솔 출력만 |
| `plot_session_grid.py` / `plot_combined_summary.py` | 세션 CSV → yaw별 격자 PNG / 여러 세션 → 모자이크+겹침 PNG. **miniconda `base` 환경 python 필요**(`~/miniconda3/bin/python3`, px4sim/pinn_train 아님) | - | `python3 plot_session_grid.py <csv> [--out-dir DIR]` | `<out-dir>/*.png` |

### 옵션이 있는 스크립트

`wind_random_sweep.py`는 `--n-yaw`(yaw 그리드 개수, 기본 24), `--n-per-yaw`(yaw
하나당 바람조건 수, 기본 10), `--yaw-offset`(그리드 시작점을 밀기, 기본 0),
`--trial-duration`(조건당 관측 시간(초), 기본 8), `--speed-min`/`--speed-max`/`--seed` 지원:

```bash
python wind_random_sweep.py --n-per-yaw 20                          # 대규모 수집 예시 (24x20=480조건)
python wind_random_sweep.py --n-yaw 6 --n-per-yaw 10 --speed-min 4 --speed-max 11
python wind_random_sweep.py --n-yaw 12 --n-per-yaw 15 --trial-duration 15   # 조건당 관측시간 늘리기
```

`run_yaw_collection_sessions.sh`는 위 옵션 대부분을 그대로 받아서 세션마다 반복
(`--yaw-offset`은 세션마다 자동 계산되므로 직접 안 줘도 됨):

```bash
./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15
# -> 2도 간격, 약 13시간 (실행 시 콘솔에 예상 소요시간을 미리 출력해줌)

# 백그라운드로 오래 돌리고 실시간 로그를 파일로 남기려면:
nohup ./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15 \
  > ../../logs/collection_run.log 2>&1 &
disown
tail -f ../../logs/collection_run.log

# 중간에 끊겨서 특정 세션부터 이어서 돌리려면 (offset/seed는 --sessions 기준으로 계산되므로
# --sessions는 원래 값 그대로 유지하고 --start-session만 바꿀 것):
./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15 --start-session 7

# 세션마다 자동으로 만드는 PNG가 필요 없으면:
./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15 --no-plot
```

**같은 명령을 실수로 두 번 실행하면 SITL/포트를 두 프로세스가 나눠 쓰면서 서로
충돌**하니, 재시작 전엔 항상 `pgrep -af "run_yaw_collection_sessions.sh"`로 이미 떠
있는 게 없는지 확인할 것. 세션이 원인 불명으로 계속 멈추면(연결은 되는데 GPS/홈 위치
확인에서 안 넘어감) `parameters.bson` 오염이 1순위 의심 대상 — setup_guide.md
트러블슈팅 표 참고.

`watch_collection.sh`는 `--log`/`--logs-dir`/`--stall-min`/`--interval-s` 지원
(기본값은 `run_yaw_collection_sessions.sh`와 같은 디렉터리에서 실행한다고 가정):

```bash
./watch_collection.sh --stall-min 5 --interval-s 60
```

`wind_gust_sweep.py`는 `wind_random_sweep.py`와 동일하게 `--n-yaw`/`--n-per-yaw`/
`--yaw-offset` 지원 (기본 12x5=60개), `--episode-duration`(에피소드당 관측 시간(초),
기본 20)으로 조건당 관측시간 조절 가능, `--speed-min`/`--speed-max`/`--seed`도 지원:

```bash
python wind_gust_sweep.py --n-per-yaw 8 --speed-min 4 --speed-max 10 --seed 456
```

CSV와 마찬가지로 PNG도 실행 하나당 `figures/wind_gust_TIMESTAMP_yaw..._n..._speed..._seed.../`
폴더 하나에 정리됨 (오케스트레이션 스크립트가 없어서 끄는 옵션은 따로 없음).

`wind_yaw_generalization_test.py`:

```bash
python wind_yaw_generalization_test.py --yaw 180 --n 10   # yaw=180도에서 10개 조건
```

---

## sim_scripts/correction_experiments/

학습된 PINN을 실제 비행 제어(가속도 피드포워드 보정)에 연결해서 효과를 검증/튜닝하는
스크립트들. 전부 `torch` 필요 (`offline_training/wind_pinn_model.py`의 `WindPINN`을
로드해서 씀).

| 파일 | 용도 | 필요 SITL 월드 | 실행 명령 | 결과 저장 위치 |
|---|---|---|---|---|
| `pinn_wind_correction_sweep.py` | 5개 바람 조건(calm~crosswind) OFF/ON A/B | `gz_x500_windy` 필수 | `python pinn_wind_correction_sweep.py` | `../../logs/pinn_correction_sweep_TIMESTAMP.csv` |
| `pinn_wind_correction_gust_sweep.py` | 위의 gust 버전(4조건) | `gz_x500_windy` 필수 | `python pinn_wind_correction_gust_sweep.py` | `../../logs/pinn_correction_gust_sweep_TIMESTAMP.csv` |
| `pinn_correction_param_tuning.py` | `ACCEL_GAIN`/`WIND_DEADBAND_MPS` 스윕 (현재 값: 0.05/1.0, 12-31절) | `gz_x500_windy` 필수 | `python pinn_correction_param_tuning.py` | `../../logs/pinn_correction_param_tuning_TIMESTAMP.csv` |
| `pinn_rotation_correction_test.py` | 회전 방해토크 피드포워드 A/B — `[gain] [condition] [target_yaw]` 위치인자 | `gz_x500_windy` + 회전 피드포워드 펌웨어 패치 필요 | `python pinn_rotation_correction_test.py 0.2` | `../../logs/pinn_rotation_correction_TIMESTAMP.csv` |
| `repeat_sweep.sh` | 위 스크립트를 N회 반복(세션마다 SITL 재시작), 노이즈에 묻히지 않는 A/B용 | 스크립트가 직접 SITL을 껐다 켬 | `./repeat_sweep.sh --script pinn_wind_correction_sweep.py --repeats 5` | manifest.txt + 반복별 CSV |
| `aggregate_repeats.py` | `repeat_sweep.sh` manifest를 읽어 평균±표준편차 집계 | - | `python3 aggregate_repeats.py <manifest.txt>` | 콘솔 출력 + 요약 CSV |

위 4개 correction 스크립트는 `correction_common.py`(같은 디렉터리, `set_wind`/
스무딩/deadband/PINN 추론버퍼 공통 유틸)를 가져다 씀. `offline_training/wind_estimator.pt`는
경로로 자동 로드하므로, 배포용 모델을 갱신했으면 다시 실행할 때 자동으로 새 모델을 씀.

---

## offline_training/

| 파일 | 용도 | 실행 명령 | 결과 저장 위치 |
|---|---|---|---|
| `wind_pinn_model.py` | 물리 방정식(`physics_residual`)/모델 구조(`WindPINN`)/하이퍼파라미터(`WINDOW`, `HIDDEN` 등)만 모아둔 파일. 직접 실행하는 파일 아님 - 수식이나 하이퍼파라미터 수정은 여기서 | (import 전용, 실행 안 함) | - |
| `train_wind_estimator.py` | `wind_random_sweep.py`/`wind_gust_sweep.py`로 모은 CSV로 바람 추정 PINN(지도학습+물리 loss)을 학습 | `python train_wind_estimator.py <csv...> [--kfold N]` | `wind_estimator.pt` (`--kfold` 지정 시엔 진단용 검증만 하고 모델은 저장하지 않음) |
| `evaluate_checkpoint.py` | 저장된 `wind_estimator.pt`를 학습에 안 쓴 새 CSV로 평가 (주로 yaw 일반화 검증용, `wind_yaw_generalization_test.py`와 세트) | `python evaluate_checkpoint.py <csv...> [--model PATH]` | 콘솔 출력만 |

```bash
# 단일/여러 CSV로 배포용 모델 학습 (항상 마지막에 80/20 분할로 저장됨)
python train_wind_estimator.py ../logs/wind_random_20260808_052538_yaw0p0_n12x15_seed1042.csv
python train_wind_estimator.py ../logs/wind_random_*.csv ../logs/wind_gust_*.csv

# k-fold 교차검증만 (모델 저장 안 함, 성능 안정성 확인용)
python train_wind_estimator.py ../logs/wind_random_*.csv ../logs/wind_gust_*.csv --kfold 5
```

`wind_estimator.pt`는 이 스크립트의 산출물이며, `pinn_wind_correction_sweep.py` /
`pinn_wind_correction_gust_sweep.py`가 실행 시 자동으로 로드해서 씀 — 따로 실행할 파일은
아님.

---

## logs/

직접 실행하는 파일이 아니라 위 수집/실험 스크립트들의 결과 CSV가 쌓이는 폴더.
파일명 접두사로 어떤 스크립트가 만들었는지 구분됨 (`wind_random_*`, `wind_gust_*`,
`wind_yawtest_*`, `pinn_correction_sweep_*`, `pinn_correction_gust_sweep_*`,
`pinn_correction_param_tuning_*`).
