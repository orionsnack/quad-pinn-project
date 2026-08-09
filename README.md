# PINN 기반 쿼드콥터 외란 추정 및 제어

공학 설계 프로젝트. PX4 SITL(Gazebo) 환경에서 바람 외란을 추정하는 PINN(Physics-Informed
Neural Network)을 학습시키고, 그 추정치를 실제 비행 제어 루프에 연결해서 효과를 검증하는
프로젝트입니다.

**환경 재현 방법은 [setup_guide.md](setup_guide.md)에 있습니다.** 이 문서는 지금까지
진행한 작업 요약, 스크립트별 설명, 트러블슈팅, 실험 상세 기록까지 모두 담고 있습니다.

---

## 지금까지 한 것 (요약)

### 1. 시뮬레이션 환경 구축 & 기본 버그 해결
PX4 SITL + Gazebo(Harmonic) + MAVSDK-Python 환경을 구축하고, Offboard 제어 스크립트들을
만드는 과정에서 발견한 버그들을 해결했습니다.
- **yaw rate 명령이 비결정적으로 동작하던 버그**: 이륙 후 목표 고도 도달을 확인 안 하고
  바로 Offboard로 넘어가서, 지면 근처에 낮게 고정된 채로 yaw 명령이 무시되던 문제였음.
  `telemetry.position()`으로 실제 고도 도달을 확인한 후 Offboard 전환하도록 수정함.

### 2. 바람 외란 PID-only 베이스라인
`gz_x500_windy` 월드에서 순수 PID 컨트롤러가 바람에 얼마나 잘 버티는지 측정. 강풍
(8.5m/s)에서도 위치 오차는 5cm 이내로 거의 완벽히 잡지만, 그 대가로 roll/pitch가
바람 세기에 비례해서 계속 기울어진 채 유지됨 — 이게 PINN이 추정할 대상.

### 3. PINN 바람 추정 모델
드론의 최근 상태(수평속도, roll, pitch)만 보고 현재 바람 벡터를 추정하는 작은 신경망을
학습. 지도학습 loss + 물리 법칙(항력 방정식) 기반 loss를 함께 사용.
- **고정바람 조건(40개)만으로 학습**했을 때: 풍속 추정 오차 **0.37~0.5 m/s** (평균
  풍속 4.6m/s 대비 약 10%) — 꽤 정확함
- **시간에 따라 변하는 바람(gust)까지 포함**해서 재학습했을 때: 오차 **1.8 m/s**로
  뚜렷이 악화. 모델 크기 키우기, 정규화, 데이터 추가, 강풍 데이터 보강 등 여러 방향을
  시도했지만 못 뚫음. 5-fold 교차검증으로 재확인해도 결과는 동일 → **지금 모델 구조
  (작은 MLP + 최근 1초 히스토리)의 한계**로 잠정 결론

### 4. 추정치를 실제 제어에 연결
- **1차 시도(실패)**: 추정 바람만큼 목표 위치(position setpoint)를 반대로 밀어주는
  방식 → 보정이 setpoint를 계속 흔들면서 폐루프 발산(roll 수십도까지) 발생. "PID가
  이미 자세 트림만으로 위치오차를 거의 다 잡고 있어서 위치 보정이 애초에 안 맞는
  지렛대"였다는 걸 알게 됨.
- **2차 시도(성공)**: 목표 위치는 절대 안 흔들고, 추정 바람 반대방향으로 **가속도만
  추가로 얹어 보내는 방식**(`set_position_velocity_acceleration_ned`)으로 구조 변경.
  강풍 온셋 시 피크 위치오차가 **0.47m → 0.21m로 약 55% 감소**.
- 5개 바람 조건(무풍/약함/보통/강함/옆바람)에서 A/B 반복 검증, deadband(무풍일 때
  보정 끄기) 추가 후 **5개 조건 전부 개선**(+8.2%~+71.3%).
- **gust 조건으로 확장 검증**: 위 검증은 전부 고정바람 기준이었는데, 바람이 사인파로
  계속 출렁이는 gust 조건 4개(약함/보통/강함/옆바람 방향, 진폭 ±60%, 주기 6초)에서도
  같은 A/B를 돌려봄. gust 풍속 추정 자체가 부정확(3절 참고, MAE 1.8m/s)함에도
  **4개 중 3개 조건 개선**(보통 +33.6%, 강함 +23.5%, 옆바람 +29.7%), 약한 gust만
  소폭 악화(-5.3%) — 상세 기록은 12-7절 참고.
- **`ACCEL_GAIN`/`WIND_DEADBAND_MPS` 체계적 튜닝 시도**: 약한 gust 문제를 겨냥해
  deadband를 낮추고 gain을 올려봤으나, 전체 조건으로 재검증하니 강풍 조건은 좋아지는
  대신 무풍/약풍 조건이 그만큼 나빠지는 트레이드오프만 확인됨. **원래 손으로 정한 값
  (0.15/1.0)이 전체 조건 기준으로는 이미 가장 균형 잡힌 선택**이라는 결론 — 상세
  기록은 12-8절 참고.

---

## 결과 하이라이트

| 실험 | 지표 | 결과 |
|---|---|---|
| PID-only 베이스라인 (강풍 8.5m/s) | 정상상태 위치오차 | <5cm |
| PINN 바람 추정 (고정바람만) | 풍속 MAE | 0.37~0.5 m/s |
| PINN 바람 추정 (gust 포함, 5-fold) | 풍속 MAE | 1.79 ± 0.52 m/s |
| PINN 바람 추정 (yaw 그리드 0~358도, 2700조건) | 풍속 MAE (val) | 0.469 m/s |
| yaw 일반화 검증 (학습 그리드에 없는 각도) | wind_vx/vy MAE | 0.229 / 0.241 m/s |
| PINN 보정 ON vs OFF (강풍 1조건) | 피크 위치오차 | 0.469m → 0.212m (-55%) |
| PINN 보정 ON vs OFF (5조건 스윕, 고정바람) | 피크 위치오차 개선 | 5/5 조건 개선 (+8.2%~+71.3%) |
| PINN 보정 ON vs OFF (4조건 스윕, gust) | 피크 위치오차 개선 | 3/4 조건 개선 (-5.3%~+33.6%) |

---

## 폴더 구조

```
sim_scripts/
  data_collection/       PINN 학습용 데이터 수집 (wind_random/gust_sweep, yaw 검증, 오케스트레이션)
  correction_experiments/ PINN 보정 A/B·파라미터 튜닝
offline_training/        PINN 학습 코드 + 학습된 모델(wind_estimator.pt)
logs/                     실험 결과 CSV
figures/                  수집 실행별(시간+조건명) 폴더에 정리된 요약 PNG
  wind_random_TIMESTAMP_n{N}x{M}_td{S}/   세션별 격자 PNG + 전체 모자이크/겹침 PNG
  wind_gust_TIMESTAMP_yaw..._seed.../     gust 수집 1회분 격자 PNG
```
자세한 파일별 설명은 아래 8절 참고.

## 다음으로 할 만한 것
- **(완료) yaw 일반화용 대규모 데이터 수집 + 재학습 + 검증** — 15세션/2700조건 수집,
  학습 그리드에 없는 각도(91도)에서 검증해 MAE 0.2m/s대로 일반화 성공 확인 (12-11절)
- **(완료) 새 모델로 12-5절 기준 PINN 보정 A/B 재검증** — 4/5 조건 여전히 개선 확인
  (12-12절). 12-7절(gust A/B)/12-8절(파라미터 튜닝)은 아직 구버전 체크포인트 결과라
  재검증 남음
- gust 추정 정확도 개선 (모델 구조 변경 또는 데이터 대규모 확장) — 우선순위 낮음, 보류 중
- 풍속에 따라 `ACCEL_GAIN`/`WIND_DEADBAND_MPS`가 같이 변하는 적응형(adaptive) 보정으로
  구조 변경 — 단일 상수로는 무풍/약풍과 강풍을 동시에 최적화 못 한다는 게 12-8절에서
  확인됨
- 실제 하드웨어(Jetson + 컴패니언 컴퓨터)로 이관
- 캡스톤 보고서용 결과 정리/시각화

(상세 내용은 아래 11절 "다음 단계")

---

## 8. 작성한 테스트 스크립트들

`sim_scripts/`는 용도별로 두 하위 폴더로 나뉨: 데이터 수집은 `data_collection/`,
PINN 보정 A/B·튜닝 실험은 `correction_experiments/`. 아래 각 스크립트는 그 폴더
안에서(`cd sim_scripts/data_collection` 또는 `cd sim_scripts/correction_experiments`)
실행하는 걸 전제로 함. 실행 전 항상 WSL 다른 터미널에서 PX4 SITL(`pxh>`)이 돌고
있어야 함.

### 8-7. `wind_random_sweep.py` — PINN 학습용 무작위 바람 데이터 수집 (고정바람)

초기 `wind_sweep_baseline.py`(고정 5조건 calm/light/default/strong/crosswind로 물리
검증만 하던 스크립트, 목적 완료 후 제거 - `gz topic`으로 바람을 런타임에 바꾸는
`set_wind()` 패턴과 "바람이 강할수록 roll/pitch가 단조 증가, crosswind는 pitch 대신
roll로 쏠림"이라는 물리적으로 타당한 패턴 확인이 성과였음)보다 훨씬 다양하게: yaw(기수
방향)를 그리드로 등분하며 회전하고, 각 yaw마다 풍속 0~10m/s·방향 0~360°를 무작위로
뽑은 조건을 여러 개(고정 시드, 재현 가능) 호버링하며 상태(속도/자세)를 20Hz로 기록.
yaw도 그리드로 도는 이유는 12-10절 참고 (roll/pitch가 yaw에 종속적이라 다양한 yaw로
안 모으면 모델이 특정 방향에서만 통함). `gz topic`으로 직접 설정한 값이라 **정답
바람벡터를 100% 정확히 앎** → 지도학습 라벨로 그대로 사용 가능. 결과는
`../../logs/wind_random_TIMESTAMP_yaw{offset}_n{yaw수}x{yaw당조건수}_speed{범위}_seed{시드}.csv`.

`--n-yaw`(yaw 그리드 개수, 기본 24), `--n-per-yaw`(yaw 하나당 무작위 바람 조건 수,
기본 10), `--yaw-offset`(그리드 시작점을 밀기, 기본 0), `--trial-duration`(조건당
관측 시간(초), 기본 8), `--speed-min`/`--speed-max`/`--seed` 옵션 지원.

```bash
python wind_random_sweep.py
python wind_random_sweep.py --n-yaw 6 --n-per-yaw 10 --speed-min 4 --speed-max 11   # 강풍 위주 추가 수집 예시
```

### 8-7-1. `wind_gust_sweep.py` — PINN 학습용 gust(시간에 따라 변하는 바람) 데이터 수집

`wind_random_sweep.py`는 한 트라이얼 내내 바람이 고정이었음. 이건 에피소드마다
`vx(t)=base+amp*sin(2πt/period+phase)` 형태로 바람이 계속 변하도록 해서, "지금 이
순간의 바람을 실시간 추적"하는 걸 배우게 하는 데이터. Gazebo에는
`GUST_UPDATE_INTERVAL_S`(1.0초) 간격으로만 갱신해서 보내지만(계단식 근사, 너무 자주
호출하면 `gz topic pub` 프로세스 spawn 비용 때문에 확 느려짐 - 처음 0.25초로 했다가
26분 걸려서 1.0초로 늘림), CSV 라벨은 매 로그 순간의 정확한 연속함수 값을 씀.
`wind_random_sweep.py`와 동일하게 yaw 그리드(`--n-yaw`/`--n-per-yaw`/`--yaw-offset`,
기본 12x5=60개, 그리드로 도는 이유는 8-7절/12-10절과 동일)로 돌며 `--episode-duration`
(에피소드당 관측 시간(초), 기본 20)/`--speed-min`/`--speed-max`/`--seed`도 지원.
결과 CSV 파일명에도 이 값들이 그대로 들어감(예:
`wind_gust_TIMESTAMP_yaw0p0_n12x5_speed1-8_seed123.csv`).

수집이 끝나면 `plot_session_grid.py`(8-7-3절)를 자동으로 호출해서 격자 PNG를
`../../figures/wind_gust_TIMESTAMP_..._seed.../`에 저장함 (오케스트레이션 스크립트가
없어서 `run_yaw_collection_sessions.sh`처럼 끄는 옵션은 따로 없음).

```bash
python wind_gust_sweep.py
python wind_gust_sweep.py --n-per-yaw 8 --speed-min 4 --speed-max 10 --seed 456   # 강풍 gust 추가 수집 예시
```

### 8-7-2. `run_yaw_collection_sessions.sh` — yaw 그리드 데이터 수집 오케스트레이션

`wind_random_sweep.py`(8-7)를 yaw 오프셋을 바꿔가며 여러 세션으로 나눠 반복 실행하는
스크립트. 세션마다 SITL을 완전히 재시작하고, yaw 그리드 시작점을 세션마다 조금씩
밀어서 합쳤을 때 훨씬 촘촘한 그리드가 되도록 설계됨 (자세한 설계 이유는 스크립트
상단 주석과 12-10절 참고).

세션마다 생성되는 CSV 파일명에 타임스탬프뿐 아니라 yaw offset/그리드 크기/시드도
들어가서(예: `wind_random_20260808_231716_yaw12p0_n12x15_seed7042.csv`), 파일명만 보고도
어느 세션(어느 offset)인지 구분 가능함.

실행마다 `figures/wind_random_TIMESTAMP_n{yaw수}x{yaw당조건수}_td{트라이얼길이}/`
폴더를 하나 만들고, 세션 하나가 끝날 때마다 `plot_session_grid.py`를 자동으로
호출해서 그 세션의 12개 yaw × 조건당 위치오차 시계열을 3x4 격자 PNG로 그 폴더 안에
저장함. 모든 세션이 끝나면 `plot_combined_summary.py`를 한 번 더 호출해서 세션별
격자 PNG를 5열 모자이크(`all_sessions_montage.png`) 하나로 합치고, 전체 세션의 모든
트라이얼을 원본 CSV에서 다시 읽어 하나의 그래프에 겹친 `all_trials_overlay.png`도
같은 폴더에 저장함 (`--no-plot`으로 이 셋 다 끌 수 있음). 아래 8-7-3절/8-7-4절 참고.

**실행 (터미널을 나중에 닫아도 계속 돌게, 로그는 실시간으로 파일에 쌓이게):**

```bash
cd ~/MyProjects/quad-pinn-project/sim_scripts/data_collection
nohup ./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15 > ../../logs/collection_run.log 2>&1 &
disown
tail -f ../../logs/collection_run.log
```

중간에 끊겨서 특정 세션부터 이어서 돌리고 싶으면 `--start-session N` 추가 (offset/seed는
전체 세션 수 기준으로 계산되므로 `--sessions`는 원래 값 그대로 유지하고 `--start-session`만
바꿀 것):

```bash
./run_yaw_collection_sessions.sh --sessions 15 --n-yaw 12 --n-per-yaw 15 --trial-duration 15 --start-session 7
```

`tail -f`로 보이는 화면은 그냥 로그를 구경하는 뷰어일 뿐이라 꺼도 수집엔 지장 없음
(nohup + disown으로 이미 터미널과 분리되어 있음). **같은 명령을 실수로 두 번 실행하면
SITL/포트를 두 프로세스가 나눠 쓰면서 서로 충돌**하니, 재시작 전엔 항상
`pgrep -af "run_yaw_collection_sessions.sh"`로 이미 떠 있는 게 없는지 확인할 것.

세션이 원인 불명으로 계속 멈추면(연결은 되는데 GPS/홈 위치 확인에서 안 넘어감),
`~/MyProjects/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson`이 오염됐을
가능성이 1순위 의심 대상임 — 지우면(또는 이름 바꿔 백업하면) PX4가 기본값으로 새로
만듦. 세션 사이 `pkill -9`로 강제종료를 반복하는 구조라 이 파일의 자기장 캘리브레이션
값이 가끔 깨지는 것으로 추정됨 (2026-08-08 밤 실제로 이걸로 세션 7이 계속 멈췄었음).

### 8-7-3. `plot_session_grid.py` — 세션 하나를 3x4 격자 PNG로 요약

세션 CSV 하나(yaw 12개 x 조건당 N개)를 읽어서, yaw별로 subplot 하나씩(3x4=12개)
만들고 그 안에 해당 yaw의 모든 조건의 위치오차 시계열을 겹쳐 그림 (선 색 = 풍속).
`run_yaw_collection_sessions.sh`가 세션이 끝날 때마다, `wind_gust_sweep.py`(8-7-1절)가
수집을 마칠 때마다 자동으로 호출하지만, 특정 CSV 하나만 다시 그리고 싶을 때 수동으로도
씀:

```bash
python plot_session_grid.py ../../logs/wind_random_TIMESTAMP_yaw12p0_n12x15_seed7042.csv
```

한글 라벨 때문에 WSL에 없는 한글 폰트가 필요해서, Windows 쪽 Noto Sans KR
(`/mnt/c/Windows/Fonts/NotoSansKR-VF.ttf`)을 직접 지정해서 씀 — 그 경로가 없는
환경에서는 경고만 찍고 기본 폰트로 넘어감(스크립트가 죽지는 않음). `matplotlib`/
`pandas`가 필요한데 `px4sim`/`pinn_train` 환경엔 없어서, 이 스크립트만 miniconda
`base` 환경 python으로 실행함 (호출하는 쪽에서 자동으로 그렇게 함).

### 8-7-4. `plot_combined_summary.py` — 여러 세션을 모자이크+전체 겹침으로 합치기

`run_yaw_collection_sessions.sh`가 세션을 전부 끝낸 뒤 자동으로 호출함. 두 가지를
만듦: (1) 세션별 격자 PNG(8-7-3절 산출물)들을 5열 모자이크 하나로 축소해서 붙인
`all_sessions_montage.png`, (2) 모든 세션의 모든 조건(위치오차 시계열)을 원본 CSV에서
직접 읽어 선 하나의 그래프에 다 겹친 `all_trials_overlay.png` (색=풍속, 선 투명도를
낮춰서 밀도로 보이게 함). 특정 PNG/CSV 묶음을 다시 합치고 싶을 때 수동 실행도 가능:

```bash
python plot_combined_summary.py --pngs ../../figures/RUN/session_*.png \
    --csvs ../../logs/wind_random_*_yaw*.csv --out-dir ../../figures/RUN
```

`plot_session_grid.py`와 마찬가지로 miniconda `base` 환경 python으로 실행.

### 8-10. `pinn_wind_correction_sweep.py` — PINN 보정 다중 조건 A/B 스윕

`pinn_wind_correction_test.py`(강풍 1개 조건 A/B, 가속도 피드포워드 방식 - 자세한
설계 이유는 12절 참고)의 다중 조건 버전. calm/light/default/strong/crosswind 5개
조건 각각에서 OFF→ON
쌍을 반복해서, 55% 개선이 특정 조건에서만 우연히 나온 게 아닌지 확인. deadband
(`WIND_DEADBAND_MPS`) 적용 후 **5개 조건 전부 개선**(+8.2%~+71.3%). 자세한 결과는
12절 참고.

```bash
python pinn_wind_correction_sweep.py
```

### 8-11. `offline_training/train_wind_estimator.py` — PINN 바람 추정 모델 학습

`wind_random_sweep.py`/`wind_gust_sweep.py`로 모은 CSV(여러 개 동시 지정 가능)를
읽어서 물리 기반(physics-informed) loss와 함께 학습. 자세한 구조는 12절 참고.
`--kfold N` 옵션으로 N-fold 교차검증(진단용, 모델 저장 안 함)도 가능 - 조건 수가
100개 안팎일 땐 한 번의 80/20 분할만으로 평가하면 결과가 크게 흔들려서 추가함
(12-6절 참고). 기본 동작(옵션 없이)은 항상 마지막에 단일 80/20 분할로 실제 배포용
모델(`wind_estimator.pt`)을 학습/저장함.

```bash
# pinn_train 환경의 python 사용 (setup_guide.md 7-1절 참고)
python train_wind_estimator.py ../logs/wind_random_TIMESTAMP.csv
python train_wind_estimator.py ../logs/wind_random_*.csv ../logs/wind_gust_*.csv --kfold 5
```

---

## 10. 트러블슈팅 모음

| 증상 | 원인/해결 |
|---|---|
| `ninja: error: unknown target 'list_vmd_make_targets'` | 첫 빌드 전 타겟 조회 명령을 실행해서 발생. `make distclean` 후 재시도하거나 무시하고 바로 빌드 |
| QGC가 "Disconnected"로 안 붙음 | WSL2와 Windows 네트워크 분리 문제. `.wslconfig`에 `networkingMode=mirrored` 설정 후 `wsl --shutdown` |
| MAVSDK `udp://` deprecated 경고 | `udpin://` 또는 `udpout://`로 명시 (`udpin://0.0.0.0:14540` 형태 권장) |
| yaw 텔레메트리 값이 실시간 반영 안 됨(계속 같은 값) | `anext()`를 필요할 때만 호출하는 방식의 스트림 적체 문제. 백그라운드 태스크로 계속 소비하며 최신값만 저장하는 방식으로 변경 |
| 같은 조건인데 A/B 테스트 결과가 이전 실행과 크게 다름(특히 calm 베이스라인이 갑자기 커짐) | 12-8절에서 **단 1회** 관측됨(SITL 65분 연속 실행 후). "장시간 실행이 원인"이라는 건 검증 안 된 가설이지 확립된 규칙이 아님 — 재현 실험은 안 해봤음. 다만 이상하게 튀는 결과가 나오면 SITL을 재시작해서 재현되는지 확인해볼 가치는 있음 |
| `make px4_sitl` 재구성 중 `kconfiglib is not installed` 에러, 심하면 `build/px4_sitl_default` 폴더 자체가 사라짐 | CMake가 Python3를 찾을 때 PATH상 `px4sim` conda 환경의 python을 잡는 경우가 있는데, 거기엔 PX4 빌드용 패키지(`kconfiglib` 등)가 없어서 재구성이 실패하며 빌드 폴더를 정리해버림. `environment-px4sim.yml`에 `PX4-Autopilot/Tools/setup/requirements.txt` 전체를 포함시켜 재발 방지함 (`conda env update -f environment-px4sim.yml`로 기존 환경에도 추가 가능) |
| `HEADLESS=1 make px4_sitl ...`를 백그라운드로 오래 돌리면 출력 파일이 수 GB까지 불어남 | PX4의 `pxh>` 콘솔이 진짜 터미널이 아닌 파이프로 연결되면 프롬프트를 계속 지우고 다시 그리는(ANSI escape) 동작을 무한 반복하는 것뿐 — SITL 자체는 정상 동작. 출력을 `> /dev/null 2>&1`로 버리고 백그라운드로 띄우면 문제없음 |

---

## 11. 다음 단계

**완료** (자세한 내용은 1절 요약과 12절 상세 기록 참고):
- 바람 외란 PID-only 베이스라인 데이터 수집 (8-5, 8-6절)
- PINN 바람 추정 모델 학습 + 실제 비행 연결 + 다중 조건 검증 (12절)
- calm(무풍) deadband 문제 해결, 5개 조건 전부 개선으로 재검증 (12-5절)
- PINN 학습 전용 conda 환경(`pinn_train`) 분리 (setup_guide.md 7-1절)
- gust 데이터 확장 시도 + k-fold 교차검증 도입 — 현재 모델 구조의 한계로 결론 (12-6절)
- gust 조건 PINN 보정 A/B 검증, 4개 중 3개 조건 개선 (12-7절)
- `ACCEL_GAIN`/`WIND_DEADBAND_MPS` 체계적 튜닝 — 원래 값이 최선으로 재확인 (12-8절)
- 배포 체크포인트가 운 나쁜 학습이었던 문제 발견/수정 (12-9절)
- yaw 종속성 문제 발견, 1차 수정(회전공식) 실패 → yaw 그리드 데이터 수집으로 재설계
  (12-10절)
- yaw 그리드 데이터(15세션, 2700조건) 수집 + 재학습 + 그리드에 없는 각도(91도)에서
  일반화 검증 성공(MAE 0.2m/s대, 1차 시도의 7~9m/s에서 크게 개선) (12-11절)
- 새 모델로 12-5절 기준 PINN 보정 A/B 재검증, 4/5 조건 여전히 개선 확인 (12-12절)

**진행 중**:
- 12-7절(gust 조건 A/B)/12-8절(파라미터 튜닝)도 아직 구버전 체크포인트로 측정된
  결과라, 새 모델로 재검증 남음 (12-5절 기준 재검증만 12-12절에서 완료됨)

**남은 것 (후보, 우선순위 미정)**:
- gust 정확도 개선하려면 모델 구조를 아예 바꾸거나(RNN/시계열류) 데이터를 지금보다
  훨씬 큰 규모로 모아야 할 것으로 보임 (12-6절 결론 참고) — 우선순위 낮음으로 보류 중
- `ACCEL_GAIN`/`WIND_DEADBAND_MPS`를 상수가 아니라 추정 풍속에 따라 값이 변하는
  적응형(adaptive) 구조로 바꾸는 것 — 12-8절에서 단일 상수의 한계가 확인됐으므로
  다음 시도할 방향은 이쪽
- MAVLink 브릿지 ↔ PINN 추론 연동을 실제 하드웨어(Jetson + 컴패니언 컴퓨터) 기준으로
  재설계 — 지금까지는 SITL 위에서 같은 Python 프로세스 안에 다 넣어서 돌린 것
- 캡스톤 보고서용으로 결과 정리/시각화
- (교훈) 학습 스크립트처럼 CPU를 많이 쓰는 작업은 SITL 비행 테스트와 동시에 돌리지
  말 것 — 한 번 IMU 타임스탬프 에러가 난 적 있음 (다행히 드론엔 문제 없었음)
- (미확정 관측) 70분간 arm/disarm을 수십 번 반복한 세션에서 calm(무풍) PID
  베이스라인이 원래 0.05~0.07m 수준이던 게 0.58m까지 튄 걸 **한 번** 봤음(12-8절).
  "장시간 실행이 원인"이라는 건 이 한 번의 관측에서 나온 가설일 뿐 재현 검증은
  안 해봤음 — 규칙으로 단정하지 말 것. 다만 결과가 평소와 다르게 나오면 SITL을
  재시작해서 재현되는지 확인해보는 정도는 값싼 안전장치로 해볼 만함

---

## 12. PINN 바람 추정 + 보정 실험 상세 기록

### 12-1. 목표

"바람 벡터(원인)를 직접 추정"하는 방식으로 설계함 (대안: setpoint 보정량을 바로
출력하는 end-to-end 방식도 고려했으나, 추정값 자체가 해석 가능하고 물리식과 엮기
쉬워서 이쪽을 선택). 즉:

```
상태 이력(속도/자세) --[PINN]--> 추정 바람벡터 --[물리식/게인]--> 보정량 --> Offboard 송신
```

### 12-2. 데이터 & 모델

- **데이터**: `wind_random_sweep.py`로 무작위 40개 바람 조건(0~10m/s, 0~360°) 수집.
  `gz topic`으로 우리가 직접 설정한 값이라 정답 바람벡터를 100% 정확히 앎 (지도학습 가능).
- **입력 feature**: 최근 10스텝(0.5초, 20Hz) 윈도우의 `[vn, ve, roll, pitch]`.
  (`pos_err`(위치오차)는 처음에 feature로 넣었다가 뺐음 — 12-3절 참고)
- **모델**: 작은 MLP (window*feature -> 64 -> 64 -> 2), 출력은 바람벡터
  `[wind_vx_enu, wind_vy_enu]` (Gazebo world 표기 그대로).
- **물리 제약(physics-informed loss)**: 항력 방정식
  `a_drag = k * |v_rel| * v_rel` (`v_rel = wind_ned - v_drone_ned`)로 계산한 예측
  가속도가 실측(속도 유한차분) 가속도와 맞도록 하는 항을 data loss에 추가.
  `k`(=0.5·ρ·Cd·A/m 묶음항)는 상수를 미리 정하지 않고 학습 가능한 파라미터로 둬서
  데이터로부터 스스로 찾게 함 → 최종 `k ≈ 0.089~0.096`.
  > 주의: Gazebo world wind는 ENU(x=East,y=North), 드론 로컬좌표는 PX4 NED(x=North,
  > y=East) 라서 물리 잔차 계산 시 축을 바꿔줘야 함 (`wind_pinn_model.py`의
  > `physics_residual()` 참고 - 방정식/모델/하이퍼파라미터는 전부 이 파일에 모여있음).
- **학습 결과** (미학습 바람조건 8개로 검증): 풍속(speed) MAE **0.37~0.50 m/s**
  (평균 실제 풍속 4.6m/s 대비). 과적합 방지를 위해 마지막 epoch이 아니라 validation
  MAE가 가장 낮았던 시점의 체크포인트를 저장하도록 함.
- **모델 파일**: `offline_training/wind_estimator.pt` (가중치 + 입력 정규화 통계
  `X_mean`/`X_std` + `window`/`features` 설정까지 같이 저장돼서, 추론 스크립트가
  그대로 로드해서 씀).

### 12-3. 1차 시도(실패): position setpoint 보정 → 폐루프 발산

**설계**: 추정 바람에 `-GAIN`을 곱해서 position setpoint(목표 위치)에 더함
("바람 반대방향으로 목표를 살짝 밀어서 PID가 더 일찍 반응하게 유도").

**결과**: roll이 -18.6°, 위치오차 18.6m까지 발산.

**근본 원인**: 학습 데이터에서 `pos_err`는 항상 "순수하게 바람 때문에 생긴 오차"였는데,
보정을 켜면 `pos_err`의 일부가 **"보정 자신이 setpoint를 흔들어서 생긴 추종 지연"**이 됨
→ 모델이 이걸 "바람이 더 세졌다"고 오인 → 보정을 더 키움 → 지연이 더 커짐 → 양의
피드백으로 발산. 게다가 이전 `wind_disturbance_baseline.py` 실험에서 이미 PID가
자세(roll/pitch) 트림만으로 정상상태 위치오차를 5cm 이내로 잡고 있다는 게 확인된
상태라, position setpoint를 흔드는 방식 자체가 애초에 이 문제에 안 맞는 지렛대였음.

`pos_err` feature 제거, 보정량 클램핑, EMA 스무딩을 다 적용해봐도 baseline(0.43m)을
못 넘어서 **position 보정 방식 자체를 포기**.

### 12-4. 2차 시도(성공): 가속도 피드포워드로 구조 변경

**설계 변경**: MAVSDK의 `set_position_velocity_acceleration_ned()`를 사용해서
position/velocity setpoint는 **처음 고정한 값 그대로 절대 움직이지 않고**, 추정
바람의 반대방향으로 **가속도(accel_n, accel_e)만 추가로** 실어 보냄. PX4가 이
가속도를 자기 위치제어 출력에 더해서 씀 → "어디로 갈지"는 안 흔들리고, "가는 길을
옆에서 밀어주는" 역할만 함 → 폐루프 발산 문제가 구조적으로 안 생김.

```python
accel_n = clip(-ACCEL_GAIN * wind_estimate_north, -MAX_ACCEL, MAX_ACCEL)
accel_e = clip(-ACCEL_GAIN * wind_estimate_east,  -MAX_ACCEL, MAX_ACCEL)
# position/velocity setpoint는 고정값 그대로, accel만 매 사이클 갱신
```

`ACCEL_GAIN=0.15`, `MAX_ACCEL_MPS2=2.0`은 실측 roll/pitch 트림으로부터 역산한
`g*tan(tilt)` ≈ 0.5~3 m/s² 범위를 크게 못 벗어나게 보수적으로 잡은 값 (엄밀히 유도한
값은 아님 — 11절 "남은 것" 참고).

**단일 조건(강풍 8.5m/s) 결과**: 바람 온셋 시 피크 위치오차 0.469m → 0.212m
(**약 55% 감소**), roll/pitch는 정상 범위 유지, 발산 없음.

### 12-5. 다중 조건 검증 결과 & 한계

`pinn_wind_correction_sweep.py`로 5개 조건에서 OFF/ON 쌍을 반복 (조건당 15초).

**1차 결과** (deadband 없이): 5개 중 4개 조건에서 개선, calm(무풍)만 -67.5% (절대값은
0.043m→0.072m로 원래 작은 수준이지만, 추정 잡음만으로 괜히 보정이 걸려서 손해봄).

**deadband 추가 후 재검증**: 추정 풍속이 `WIND_DEADBAND_MPS`(1.0 m/s) 이하면 보정을
0으로, 그 이상은 문턱값에서 뺀 만큼만 부드럽게(방향 유지, 크기만 선형으로) 적용하도록
`apply_deadband()` 추가:

```python
def apply_deadband(wind_n, wind_e, deadband):
    speed = (wind_n**2 + wind_e**2) ** 0.5
    if speed <= deadband:
        return 0.0, 0.0
    scale = (speed - deadband) / speed   # 문턱값 근처에서 뚝 끊기지 않고 이어짐
    return wind_n * scale, wind_e * scale
```

| 조건 | 풍속(m/s) | OFF 피크오차 | ON 피크오차 | 개선율 |
|---|---|---|---|---|
| calm | 0.00 | 0.067m | 0.061m | +8.2% |
| light | 2.24 | 0.124m | 0.081m | +34.6% |
| default | 5.39 | 0.336m | 0.189m | +43.8% |
| strong | 8.54 | 0.585m | 0.375m | +35.9% |
| crosswind | 6.00 | 0.542m | 0.156m | +71.3% |

**5개 조건 전부 개선** (+8.2% ~ +71.3%). 바람이 강할수록/방향이 순수 옆바람에 가까울수록
효과가 큰 경향은 그대로 유지됨.

**한계 / 아직 안 한 것**:
- (12-6절에서 gust까지 다룸)
- `ACCEL_GAIN`, `WIND_DEADBAND_MPS` 둘 다 손으로 정한 값이지 최적화한 게 아님
- 전부 SITL 안에서, 하나의 Python 프로세스가 MAVSDK 연결 + 모델 추론을 동시에 처리하는
  구조. 실제 하드웨어(Jetson + 컴패니언 컴퓨터, 별도 MAVLink 링크)로 옮길 때는 추론
  지연시간, 통신 주기 등을 다시 검증해야 함

### 12-6. gust(시간에 따라 변하는 바람) 확장 시도 - 진짜 어려운 문제였음

지금까지(12-2절)는 한 트라이얼 내내 바람이 고정이었음. `wind_gust_sweep.py`로 사인파
형태 바람 데이터를 추가 수집해 기존 고정바람 데이터와 합쳐 재학습을 시도함.

**시행착오**: gust 데이터를 합쳐 재학습하니 val_MAE가 0.37~0.5 → 1.82m/s로 악화. 모델을
키워봐도(hidden 128, window 20) 전형적인 과적합만 나고(best_epoch=4, 학습 초반이 제일
나았음), weight decay를 추가해도 개선 없음. 조건별 오차를 까보니 강풍 데이터 부족이
의심돼 강풍 위주로 추가 수집해봤지만 나쁜 조건이 오히려 늘어남. 풍속-오차 상관계수도
0.578로 애매해서 "고풍속만 어렵다"로 깔끔히 설명 안 됨. **여기서 깨달은 것**: 조건이
100개 안팎일 때 한 번의 무작위 80/20 분할만으로 평가하면 그 분할에 따라 결과가 크게
흔들려서, 모델/데이터를 바꾼 효과인지 그냥 평가가 불안정한 건지 구분이 안 되고 있었음.

**해결책**: `train_wind_estimator.py`에 `--kfold N` 옵션 추가. 조건을 N등분해서 N번
학습/검증을 반복하고 평균±표준편차를 냄 (기본 동작은 그대로 유지: 마지막엔 항상 단일
80/20 분할로 실제 배포용 모델도 학습/저장함).

```bash
python train_wind_estimator.py <csv...> --kfold 5
```

**5-fold 검증 결과** (전체 데이터: 고정바람 75개 + gust 27개 = 102조건):

| 지표 | 평균 ± 표준편차 | fold별 범위 |
|---|---|---|
| wind_vx MAE | 1.930 ± 0.437 m/s | 1.21~2.55 |
| wind_vy MAE | 2.146 ± 0.437 m/s | 1.37~2.72 |
| 풍속(speed) MAE | 1.794 ± 0.520 m/s | 1.13~2.54 |

5번 평균을 내도 여전히 ~1.8m/s대로 나옴 → **"운 나쁜 분할" 때문이 아니라, gust를
포함하면서 과제 자체가 진짜로 더 어려워진 게 맞다**는 결론. (다만 fold별 편차
0.52는 여전히 커서, 조건을 더 늘리면 추정치 자체가 더 안정될 여지는 있음)

**최종 결론**: 최초(고정바람만, 40조건) 때 0.37~0.5m/s였던 게, gust+강풍을 넣으면서
1.8m/s(평균풍속 7m/s 기준 약 26%)로 뚜렷이 나빠짐. 모델 크기 조정, 정규화, 데이터
추가 등 여러 방향을 시도했지만 뚜렷한 돌파구는 못 찾음 - **지금 구조(작은 MLP + 최근
1초 히스토리)로는 gust를 포함한 정확한 실시간 바람 추정에 한계가 있다**는 걸로
잠정 정리. 더 파고들려면 모델 구조를 아예 바꾸거나(RNN/시계열 모델 등) 데이터를
지금보다 훨씬 큰 규모로 모아야 할 것으로 보임.

### 12-7. gust 조건에서의 PINN 보정 A/B 검증

**배경**: 12-5의 A/B 검증(5개 조건 전부 개선, +8.2%~+71.3%)은 전부 트라이얼 내내
바람이 고정인 조건에서만 했음. 그런데 12-6에서 확인했듯 gust가 섞이면 풍속 추정
오차 자체가 0.37~0.5m/s → 1.8m/s로 뚜렷이 나빠짐. 추정이 이 정도로 부정확한
상황에서도 가속도 피드포워드 보정이 여전히 순이익인지, 아니면 잘못된 추정이
오히려 상황을 악화시키는지 확인이 안 된 상태였음.

**방법**: `pinn_wind_correction_gust_sweep.py` 작성. 12-5와 같은 4개 방향
(light/default/strong/crosswind — calm은 base_speed=0이라 방향이 정의 안 돼서 gust
버전에서는 제외)을 그대로 쓰되, 풍속 크기를 `speed(t) = base*(1 + 0.6*sin(2π t/6))`
형태로 진폭 ±60%, 주기 6초로 계속 흔듦(`wind_gust_sweep.py`와 동일하게 `gz topic`은
1초 간격으로만 갱신). 트라이얼 길이는 주기의 3배인 18초. 나머지(보정 OFF→ON 순서,
`ACCEL_GAIN=0.15`, `WIND_DEADBAND_MPS=1.0`, EMA 스무딩 등)는 12-5와 완전히 동일하게
유지해서 고정바람 결과와 나란히 비교 가능하게 함.

**결과**:

| 조건 | base 풍속(m/s) | OFF 피크오차 | ON 피크오차 | 개선율 |
|---|---|---|---|---|
| light_gust | 2.24 | 0.148m | 0.156m | -5.3% |
| default_gust | 5.39 | 0.346m | 0.230m | +33.6% |
| strong_gust | 8.54 | 0.601m | 0.459m | +23.5% |
| crosswind_gust | 6.00 | 0.459m | 0.323m | +29.7% |

**해석**: 4개 중 3개 조건 개선(**+23.5%~+33.6%**). 고정바람(12-5, 5/5 전부 개선)보다는
못하지만, 추정이 부정확한 gust 상황에서도 중간~강풍대에서는 보정이 대체로 순이익임을
확인. `light_gust`만 소폭 악화(-5.3%)했는데, base 2.24m/s에 진폭 ±60%면 순간 풍속이
0.9~3.6m/s 사이를 오가는 구간이라 `WIND_DEADBAND_MPS`(1.0m/s) 문턱을 계속 넘나들면서
추정 잡음이 deadband로 완전히 걸러지지 않은 것으로 보임 — 12-5에서 calm(무풍)에
deadband를 추가해 해결했던 것과 비슷한 성격의 문제가 약한 gust 대역에서도 남아있는
것. 자세한 CSV 로그(추정 바람 vs 실제 바람의 시간에 따른 변화)는
`logs/pinn_correction_gust_sweep_*.csv`에 저장됨.

**한계 / 다음에 볼 것**:
- `WIND_DEADBAND_MPS`를 gust 상황에 맞게 재튜닝하면 `light_gust` 조건도 개선으로
  돌아설 가능성 있음 (11절 "남은 것" 참고)
- gust 방향/진폭/주기 조합을 더 늘려서(지금은 4개 조건, 조건당 1회) 통계적으로
  더 탄탄하게 확인할 여지는 있음

### 12-8. `ACCEL_GAIN`/`WIND_DEADBAND_MPS` 체계적 튜닝 시도

**배경**: `ACCEL_GAIN=0.15`, `WIND_DEADBAND_MPS=1.0`은 손으로 정한 값이었고, 12-7에서
약한 gust가 deadband를 못 뚫고 손해 보는 문제가 드러나 체계적으로 스윕해봄.

**1차 프로브** (`pinn_correction_param_tuning.py`, one-factor-at-a-time):

| 스윕 | 값 | 결과 |
|---|---|---|
| deadband (light_fixed) | 0.5 / 1.0 / 1.5 / 2.0 | +49.8% / +56.0% / +34.2% / +56.8% |
| deadband (light_gust) | 0.5 / 1.0 / 1.5 / 2.0 | +33.7% / +20.5% / **-5.5%** / +28.8% |
| gain (default_fixed) | 0.05/0.10/0.15/0.20/0.30 | +35.0%/+41.0%/+58.3%/**+62.8%**/-11.8% |
| gain (strong_fixed) | 0.05/0.10/0.15/0.20/0.30 | +23.2%/+56.1%/+47.5%/**+69.9%**/+67.5% |

`deadband=0.5`, `gain=0.20`이 제일 좋아 보였음(0.30은 roll이 불안정).

**2차 전체 재검증 중 SITL 오염 발견**: 새 값을 반영해 5조건 전체를 돌렸더니 calm의
OFF 베이스라인이 평소 <7cm에서 **0.580m**로 튐 — SITL이 70분 넘게 재시작 없이 연속
실행된 게 원인으로 의심됨(10절 트러블슈팅 표 참고). SITL을 재시작하고 깨끗한 상태에서
재검증.

**3차 깨끗한 SITL 재검증**:

| 조합 | calm | light | default | strong | crosswind | 개선된 조건 |
|---|---|---|---|---|---|---|
| gain=0.15, deadband=1.0 (원래 값, 12-5) | +8.2% | +34.6% | +43.8% | +35.9% | +71.3% | **5/5** |
| gain=0.20, deadband=0.5 | **-98.0%** | +15.1% | +55.0% | +48.2% | +43.5% | 4/5 |
| gain=0.20, deadband=1.0 | -15.7% | **-27.6%** | +30.3% | +42.6% | +30.6% | 3/5 |

**결론**: gain을 올리거나 deadband를 낮추면 강풍은 좋아지지만 무풍/약풍이 나빠지는
반대 방향 트레이드오프라 전역 상수로는 동시에 못 잡음. 세 조합 중 **원래 값(0.15/1.0)만
5개 조건 전부 개선**이라 그대로 유지. 진짜 다음 단계는 상수 튜닝이 아니라 **추정
풍속에 따라 gain/deadband가 같이 변하는 적응형 구조**(11절 "남은 것" 참고).

### 12-9. 배포된 체크포인트가 알고 보니 "운 나쁜 학습"이었던 문제

**발견**: 커밋돼 있던 `wind_estimator.pt`의 메타데이터를 확인해보니
**`best_val_mae=2.83m/s`, `best_epoch=5`**로, 12-6절 5-fold 평균(1.79±0.52m/s)보다
뚜렷이 나쁨. **원인**: 배포용 학습은 train/val 조건 분할은 시드 고정이지만 **가중치
초기화/미니배치 순서는 고정 안 됨**이라 재학습마다 결과가 다름 — 커밋된 체크포인트는
이 무작위성에서 유독 안 좋게 뽑힌 한 번이었음. 같은 데이터로 두 번 재학습해보니
val_MAE 2.030/2.034로 재현 잘 되고 5-fold 오차범위 안에 들어와서, 이 결과로
`wind_estimator.pt`를 갱신함.

**중요**: 12-7/12-8절 결과는 전부 이 더 안 좋은 체크포인트(2.83m/s)로 측정된 것이라,
갱신된 모델(2.03m/s)로 다시 돌리면 수치가 달라질 수 있음(재검증 안 함). 앞으로는
배포용 모델 저장 전 `best_val_mae`를 콘솔 로그뿐 아니라 체크포인트에서도 꼭 확인할 것.

### 12-10. yaw 일반화: 1차 수정(회전공식) 실패, 데이터 기반으로 재설계

**배경**: 3절의 MLP/PINN 구조 개선 후보 중 "roll/pitch가 yaw에 종속적인 문제"를
가장 먼저 다룸. `vn_m_s`/`ve_m_s`는 관성(NED) 좌표계라 문제없지만, `roll_deg`/
`pitch_deg`는 기체 좌표계라 학습 데이터의 yaw가 어디였는지에 암묵적으로 묶임 —
그런데 지금까지 모든 학습 데이터(`wind_random_*.csv`, `wind_gust_*.csv`)가
SITL 스폰 방향(75~93도) 근처에만 몰려있었음.

**1차 시도(실패, 확실함)**: roll/pitch가 yaw에 대해 대칭으로(같은 비중으로) 회전한다고
손으로 가정하고 `body_tilt_to_inertial()`(tilt_north/tilt_east 2개로 축약)을
유도해서 재학습함 — 이 자체는 잘 됐다고 착각할 뻔했는데, `wind_yaw_generalization_test.py`
로 yaw=0도(학습 범위 밖)에서 소규모(8조건) 검증 데이터를 따로 모아서
`evaluate_checkpoint.py`로 평가해보니 **wind_vx MAE 7.49, wind_vy MAE 8.78**
(정상 2.0~2.3 대비 3~4배)로 완전히 실패 — 이건 직접 측정한 확실한 결과.

**왜 실패했는지(원인, 불확실함)**: yaw=92도/0도 데이터를 합쳐 직접 회귀분석해보니
roll 축과 pitch 축의 게인이 비대칭인 것처럼 보이는 패턴이 나오긴 했으나, **이 회귀
자체가 yaw 표본이 딱 2종류뿐이라 신뢰도가 낮음** (2점만으로는 그 사이 함수 형태를
제대로 못 정함 - 다른 원인이었을 수도 있음). "왜 실패했는지"는 확정하지 않고,
"대칭 가정 방식은 실패했다"는 것만 확실한 결론으로 취급함 — 그래서 2차 시도에서는
원인을 더 파고들지 않고 그냥 신경망이 데이터로부터 알아서 배우게 하는 방향으로 감.

**중요 — 이게 "데이터셋이 이상해서" 생긴 문제인지 정리**:
- 이 yaw 일반화 실패는 **진짜 데이터(수집 설계) 문제가 맞음** — 학습 데이터의 yaw
  범위가 75~93도로 너무 좁아서 애초에 "정확한 회전 공식"을 데이터로 검증할 방법이
  없었음.
- 반면 12-9절의 체크포인트 val_MAE 문제(2.83 → 2.03)는 **데이터셋 문제가 아님** —
  같은 데이터로 재학습해도 일관되게 2.0대가 나왔으므로 랜덤 초기화 문제였음.
- 트러블슈팅 표의 SITL 드리프트(calm 베이스라인 폭증) 문제도 **원래 학습 데이터와는
  별개** — 그건 오늘 세션의 A/B 보정 테스트 쪽에서 나온 문제고, `wind_random_*.csv`/
  `wind_gust_*.csv`가 실제로 그 드리프트를 겪었는지는 확인된 바 없음(다른 세션에서
  수집됨). 세 가지를 뭉뚱그려 "데이터셋이 이상하다"고 하면 부정확함.

**2차 시도(현재, 데이터 기반)**: 정확한 결합 공식을 다시 손으로 유도하는 대신,
`roll_deg*cos(yaw)`, `roll_deg*sin(yaw)`, `pitch_deg*cos(yaw)`, `pitch_deg*sin(yaw)`
4개 항(`yaw_decompose()`)을 feature로 주고, 정확한 결합 방식은 신경망이 데이터로부터
직접 배우게 함 — 단, 이러려면 학습 데이터의 yaw 자체가 다양해야 함. 그래서:
- `wind_random_sweep.py`를 확장: yaw를 12~24개(그리드)로 회전해가며 각 방향에서
  무작위 바람 조건을 수집하도록 변경. `--yaw-offset` 옵션 추가.
- `run_yaw_collection_sessions.sh` 신설: 세션 하나를 너무 길게(그리드를 너무
  촘촘하게) 잡는 대신, 짧은 세션 여러 개를 세션마다 SITL을 재시작하며 이어 돌리고,
  세션마다 yaw 그리드 시작점(offset)을 조금씩 밀어서 합쳤을 때 훨씬 촘촘한 그리드가
  되도록 설계 (예: 12방향 그리드를 15세션 x offset 2도씩 = 합산 180지점/2도 간격).
  세션을 짧게 쪼개는 이유는 SITL 드리프트 가설(트러블슈팅 표 참고, 미검증) 때문만은
  아니고, 그거와 별개로 세션 하나가 죽어도 데이터 유실이 적고 중간 점검이 쉬워지는
  이점이 있어서 채택함.
- CSV는 조건마다 flush하도록 변경(장시간 무인 수집 중 죽어도 데이터 유실 최소화).

**현재 상태 (수집 완료)**: yaw 그리드 데이터 수집(15세션, 2700조건, yaw 0~358도
2도 간격) 완료. 재학습 및 일반화 검증 결과는 12-11절 참고.

### 12-11. yaw 그리드 재학습 결과: 일반화 성공 확인

**데이터**: 15세션 yaw 그리드 CSV(2700조건, yaw 0~358도 2도 간격)로 재학습.
`window=20`, feature는 12-10절의 `yaw_decompose()` 6개(`vn`/`ve` +
`roll_cos_yaw`/`roll_sin_yaw`/`pitch_cos_yaw`/`pitch_sin_yaw`) 그대로.

**학습 결과**: `best_epoch=28`(예전 4~5보다 훨씬 늦게 최적점 도달 - 데이터가 약
25배 늘면서 과적합이 지연된 것으로 보임), `best_val_mae=0.469m/s`(무작위 검증조건
기준) — 좁은 yaw로만 학습했던 최초 결과(0.37~0.50m/s, 12-2절)와 비슷한 수준을
**전체 yaw 범위에서** 달성함.

**일반화 검증**: `wind_yaw_generalization_test.py --yaw 91 --n 10`으로 학습
그리드에 없는 각도(91도, 그리드 점 90/92도 사이)에서 검증 데이터를 모은 뒤
`evaluate_checkpoint.py`로 평가.

| 지표 | 12-10절 1차 시도(실패, yaw=0도) | 오늘(yaw=91도, 새 모델) |
|---|---|---|
| wind_vx MAE | 7.49 m/s | **0.229 m/s** |
| wind_vy MAE | 8.78 m/s | **0.241 m/s** |

학습 그리드에 없는 각도인데도 오히려 학습 시 val_MAE(0.469)보다 좋은 성능이 나옴 —
**yaw_decompose 기반 재설계가 실제로 yaw 일반화 문제를 해결했다는 게 확인됨**.

**아직 안 한 것**: 12-7절(gust A/B)/12-8절(파라미터 튜닝) 결과는 여전히 이전(더
안 좋았던) 체크포인트로 측정된 것이라, 이 새 모델로 재검증이 남아있음.

### 12-12. 새 모델로 PINN 보정 A/B 재검증

`pinn_wind_correction_sweep.py`를 새 yaw 그리드 모델(`wind_estimator.pt`)로 재실행.
12-5절과 동일한 5개 조건·동일한 파라미터(`ACCEL_GAIN=0.15`, `WIND_DEADBAND_MPS=1.0`)로
직접 비교 가능.

| 조건 | 풍속(m/s) | 12-5절 개선율(구모델) | 오늘 개선율(새 모델) |
|---|---|---|---|
| calm | 0.00 | +8.2% | -0.3% |
| light | 2.24 | +34.6% | +41.7% |
| default | 5.39 | +43.8% | +40.3% |
| strong | 8.54 | +35.9% | **+49.2%** |
| crosswind | 6.00 | **+71.3%** | +66.7% |

개선된 조건: 4/5 (구모델은 5/5). light/default/strong/crosswind는 여전히 확실하게
개선되고(strong은 오히려 구모델보다 더 좋아짐), yaw 재학습이 보정 효과 자체를
깨뜨리지 않았다는 게 확인됨. calm만 -0.3%인데, OFF/ON 절대값이 0.041m→0.041m로
차이가 노이즈 수준(4cm 이내)이고 애초에 `WIND_DEADBAND_MPS` 때문에 무풍에선 보정이
거의 안 걸리는 게 설계 의도라, 구모델의 +8.2%(0.067→0.061m, 6mm 차이)도 실질적으로는
같은 "차이 없음" 결론이었음 — 새로운 문제가 아님.

**결론**: yaw 그리드로 재학습한 새 모델에서도 PINN 가속도 피드포워드 보정은 실제
바람이 있는 조건에서 일관되게 유효함이 재확인됨. CSV: `logs/pinn_correction_sweep_20260809_221047.csv`
