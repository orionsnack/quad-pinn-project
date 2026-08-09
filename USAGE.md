# 파일 사용법 요약

각 파일이 뭘 하는 파일이고 어떻게 실행하는지 빠르게 찾기 위한 문서입니다. 배경/설계
이유(왜 이렇게 만들었는지, 실험 결과)는 [README.md](README.md), 환경 구축은
[setup_guide.md](setup_guide.md) 참고.

**공통 전제**
- 다른 터미널에서 PX4 SITL이 `pxh>` 상태로 떠 있어야 함
  (`cd ~/MyProjects/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500` 또는 `gz_x500_windy`)
- `sim_scripts/`의 스크립트는 `conda activate px4sim` 후 실행
- `offline_training/`의 학습 스크립트는 `conda activate pinn_train` 후 실행
- 실행 위치는 항상 해당 파일이 있는 디렉터리 (`sim_scripts/` 또는 `offline_training/`)

---

## sim_scripts/

| 파일 | 용도 | 필요 SITL 월드 | 실행 명령 | 결과 저장 위치 |
|---|---|---|---|---|
| `wind_random_sweep.py` | PINN 학습용 데이터 수집 — yaw 12방향(그리드) x 방향당 무작위 바람조건(기본 10개)으로 호버링하며 라벨링. yaw도 그리드로 도는 이유는 README 12-9절/12-10절 참고 (roll/pitch가 yaw에 종속적이라 다양한 yaw로 안 모으면 모델이 특정 방향에서만 통함) | `gz_x500_windy` 필수 | `python wind_random_sweep.py` (옵션 아래 참고) | `../logs/wind_random_TIMESTAMP.csv` |
| `wind_gust_sweep.py` | PINN 학습용 데이터 수집 — 바람이 사인파로 계속 변하는 gust 조건(기본 15개 에피소드) 라벨링. yaw는 그리드로 안 돌고 스폰 방향 고정(아직 미개선) | `gz_x500_windy` 필수 | `python wind_gust_sweep.py` (옵션 아래 참고) | `../logs/wind_gust_TIMESTAMP.csv` |
| `wind_yaw_generalization_test.py` | 학습 yaw 범위와 다른 방향(기본 0도)으로 회전해서 소규모(기본 8개) 검증 데이터만 모으는 스팟체크용 — `evaluate_checkpoint.py`와 세트로 사용 | `gz_x500_windy` 필수 | `python wind_yaw_generalization_test.py` (`--yaw`, `--n` 옵션) | `../logs/wind_yawtest_{yaw}deg_TIMESTAMP.csv` |
| `pinn_wind_correction_sweep.py` | 위 A/B 테스트를 5개 바람 조건(calm~crosswind) 전체로 반복해 보정 효과의 일관성을 확인 | `gz_x500_windy` 필수 (torch 필요) | `python pinn_wind_correction_sweep.py` | `../logs/pinn_correction_sweep_TIMESTAMP.csv` |
| `pinn_wind_correction_gust_sweep.py` | 위 스윕의 gust 버전 — 바람 크기가 사인파로 계속 변하는 4개 조건에서 보정 OFF/ON A/B 비교 | `gz_x500_windy` 필수 (torch 필요) | `python pinn_wind_correction_gust_sweep.py` | `../logs/pinn_correction_gust_sweep_TIMESTAMP.csv` |
| `pinn_correction_param_tuning.py` | `ACCEL_GAIN`/`WIND_DEADBAND_MPS` 값을 여러 개 스윕하며 대표 조건에서 개선율과 roll/pitch 안정성을 비교 (결론: README 12-8절 참고, 현재 값 유지) | `gz_x500_windy` 필수 (torch 필요) | `python pinn_correction_param_tuning.py` | `../logs/pinn_correction_param_tuning_TIMESTAMP.csv` |
| `run_yaw_collection_sessions.sh` | `wind_random_sweep.py`를 여러 세션으로 나눠 반복 실행하며 세션마다 SITL을 재시작하는 bash 오케스트레이션 스크립트 — 장시간(수 시간) 데이터 수집을 단일 연속비행 대신 안전하게 나눠 돌릴 때 사용 | 스크립트가 직접 SITL을 껐다 켬 (미리 안 띄워도 됨) | `./run_yaw_collection_sessions.sh --sessions 4 --n-yaw 24 --n-per-yaw 10` | `../logs/wind_random_TIMESTAMP.csv` (세션마다 별도 파일) |

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
```

`wind_gust_sweep.py`는 `--n`(에피소드 수), `--speed-min`/`--speed-max`/`--seed` 지원:

```bash
python wind_gust_sweep.py --n 12 --speed-min 4 --speed-max 10 --seed 456
```

`wind_yaw_generalization_test.py`:

```bash
python wind_yaw_generalization_test.py --yaw 180 --n 10   # yaw=180도에서 10개 조건
```

---

## offline_training/

| 파일 | 용도 | 실행 명령 | 결과 저장 위치 |
|---|---|---|---|
| `train_wind_estimator.py` | `wind_random_sweep.py`/`wind_gust_sweep.py`로 모은 CSV로 바람 추정 PINN(지도학습+물리 loss)을 학습 | `python train_wind_estimator.py <csv...> [--kfold N]` | `wind_estimator.pt` (`--kfold` 지정 시엔 진단용 검증만 하고 모델은 저장하지 않음) |

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
`pinn_correction_ab_*`, `pinn_correction_sweep_*`).
