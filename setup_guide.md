# PX4 SITL + Gazebo + MAVSDK 시뮬레이션 환경 구축 가이드

**작성 목적**: 다른 컴퓨터에서도 동일한 시뮬레이션 개발 환경을 그대로 재현하기 위한 기록
**프로젝트**: PINN 기반 쿼드콥터 외란 추정 및 제어 - 캡스톤 설계
**검증 환경**: Windows 11 + WSL2(Ubuntu 22.04) + PX4-Autopilot + Gazebo(Harmonic) + MAVSDK-Python 3.17.2

---

## 0. 사전 준비물

- Windows 11 (WSLg를 통한 GUI 지원, WSL2 미러 네트워킹 모드 사용을 위해 필요)
- WSL2 활성화 및 Ubuntu 22.04 배포판
- 넉넉한 디스크 공간 (PX4 소스 + 서브모듈 기준 최소 10GB 이상 권장)

---

## 1. WSL2에 Ubuntu 22.04 설치

이미 WSL2가 깔려 있다면 배포판만 확인:

```powershell
wsl -l -v
```

`Ubuntu-22.04`가 목록에 없다면:

```powershell
wsl --install -d Ubuntu-22.04
```

설치 후 재부팅, 처음 실행 시 유저명/비밀번호 설정.

---

## 2. WSL2 기본 세팅

WSL 셸 진입 후:

```bash
sudo apt update && sudo apt upgrade -y
```

**반드시 홈 디렉토리(`~`)에서 작업할 것** — `/mnt/c/...` 같은 Windows 파일시스템 경로에서 작업하면 속도가 크게 느려지고 권한 문제도 발생함.

---

## 3. PX4 소스 클론 및 개발환경 설치

```bash
cd ~
mkdir -p MyProjects && cd MyProjects
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

- `--recursive` 옵션 필수 (서브모듈까지 같이 클론)
- `ubuntu.sh`가 빌드 툴체인과 Gazebo(Ubuntu 22.04 기준 최신 Harmonic 버전)를 설치함
- 완료 후 WSL 완전 재시작 필요:

```powershell
wsl --shutdown
```
그 후 다시 `wsl -d Ubuntu-22.04`로 진입.

### 3-1. 흔한 에러: `unknown target 'list_vmd_make_targets'`

첫 빌드 전에 타겟 목록 조회 명령을 실행하면 나는 에러. 무시하고 바로 실제 빌드로 넘어가거나, `make distclean` 후 재시도.

---

## 4. Gazebo SITL 첫 실행

```bash
cd ~/MyProjects/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
```

- `gz_x500`: x500 쿼드로터 모델, 최신 Gazebo(Harmonic) 사용
- `HEADLESS=1`: 3D 그래픽 없이 실행 (WSL2에서 더 안정적)
- 정상 실행되면 `pxh>` 프롬프트가 뜸 → 성공

### 4-1. 바람 있는 월드로 실행 (외란 실험용)

```bash
HEADLESS=1 make px4_sitl gz_x500_windy
```

### 4-2. 시뮬레이션 속도 조절 (필요시)

```bash
PX4_SIM_SPEED_FACTOR=0.5 HEADLESS=1 make px4_sitl gz_x500
```

### 4-3. 상태 확인 명령어 (pxh> 프롬프트에서)

```
commander status
listener sensor_combined
```

---

## 5. WSL2 네트워킹 설정 (QGroundControl 연결용)

WSL2와 Windows 간 네트워크가 분리되어 있어 기본적으로 QGC가 자동 연결 안 될 수 있음.
**미러 네트워킹 모드**로 해결.

Windows에서 (`Win + R` → `notepad %USERPROFILE%\.wslconfig`):

```ini
[wsl2]
networkingMode=mirrored
processors=8
memory=32GB
```

(`processors`, `memory`는 본인 컴퓨터 사양에 맞게 조정. 전체의 절반~2/3 정도가 무난)

저장 후:

```powershell
wsl --shutdown
```

재시작 후 PX4 SITL 다시 실행하면 `127.0.0.1`로 QGC가 바로 붙음.

---

## 6. QGroundControl 설치 (Windows 쪽)

1. https://qgroundcontrol.com/ 에서 Windows 인스톨러 다운로드
2. 설치 후 실행
3. SITL이 `pxh>` 상태로 돌고 있으면 자동 연결됨 (상단에 "비행 준비 완료" 표시되면 성공)

지도에서 우클릭 → **Go To Location** 또는 **Takeoff**로 직접 조작 가능.

---

## 7. Python 환경 및 MAVSDK 설치

```bash
# (선택) 전용 conda 환경 생성
conda create -n px4sim python=3.10 -y
conda activate px4sim

pip install mavsdk
```

프로젝트 코드는 **PX4-Autopilot 폴더 밖에 별도로 관리**할 것 (PX4는 git 저장소이므로 섞으면 안 됨):

```bash
mkdir -p ~/MyProjects/quad-pinn-project/sim_scripts
cd ~/MyProjects/quad-pinn-project/sim_scripts
```

### 폴더 구조 (실제 현재 상태)

```
~/MyProjects/
├── PX4-Autopilot/                        # 시뮬레이터 자체 (건드리지 않음)
└── quad-pinn-project/                    # 우리 프로젝트 전용 폴더
    ├── README.md                          # 프로젝트 한눈에 보기 (현재 상태/결과 요약)
    ├── setup_guide.md                     # 환경 재현 + 상세 실험 기록 (이 문서)
    ├── sim_scripts/                       # MAVSDK 테스트/실험 스크립트 (8절 참고)
    │   ├── test_connection.py
    │   ├── arm_takeoff_land.py
    │   ├── offboard_velocity_test.py
    │   ├── yaw_rate_sweep_test.py
    │   ├── wind_disturbance_baseline.py
    │   ├── wind_sweep_baseline.py
    │   ├── wind_random_sweep.py
    │   ├── wind_gust_sweep.py
    │   ├── pinn_correction_interface_test.py
    │   ├── pinn_wind_correction_test.py
    │   └── pinn_wind_correction_sweep.py
    ├── offline_training/                  # PINN 학습 파이프라인
    │   ├── train_wind_estimator.py
    │   └── wind_estimator.pt             # 학습된 모델 체크포인트
    └── logs/                              # 실험 결과 CSV (wind_random_*, pinn_correction_*)
```

> 참고: 초기 스크립트(`yaw_rate_sweep_test.py`, `wind_sweep_baseline.py` 등)는 결과 CSV를
> `sim_scripts/` 안에 그냥 저장하도록 되어 있었음. `wind_random_sweep.py`부터는
> `../logs/`에 저장하도록 통일함 - 앞으로 작성하는 스크립트도 이 규칙을 따를 것.
>
> `runtime/`(Jetson 실시간 파이프라인)은 아직 폴더도 안 만든 상태 - 11절 "남은 것" 참고.

### 7-1. PINN 학습/추론용 패키지 (conda 환경 두 개를 씀)

- **`px4sim` 환경** (시뮬레이션 + 실시간 추론용): `mavsdk` 외에 실시간으로 학습된 모델을
  돌리기 위해 CPU 버전 PyTorch도 추가 설치함.
  ```bash
  conda activate px4sim
  pip install --index-url https://download.pytorch.org/whl/cpu torch
  pip install numpy pandas
  ```
- **오프라인 학습(`offline_training/train_wind_estimator.py`)용으로 `pinn_train`이라는
  이 프로젝트 전용 환경을 따로 만듦.** (처음엔 `text2cad`라는 다른 프로젝트 환경을 급하게
  빌려썼었는데, 남의 환경이라 나중에 그쪽이 바뀌면 같이 깨질 위험이 있어서 분리함)
  ```bash
  conda create -n pinn_train python=3.10 -y
  conda activate pinn_train
  pip install --index-url https://download.pytorch.org/whl/cpu torch
  pip install numpy pandas
  ```
  실행:
  ```bash
  conda activate pinn_train
  python offline_training/train_wind_estimator.py logs/wind_random_TIMESTAMP.csv
  ```
  (모델이 작은 MLP라 CPU로도 충분히 빠름 — GPU 굳이 필요 없었음)

---

## 8. 작성한 테스트 스크립트들

모두 `sim_scripts/`에 저장. 실행 전 항상 WSL 다른 터미널에서 PX4 SITL(`pxh>`)이 돌고 있어야 함.

### 8-1. `test_connection.py` — 연결/텔레메트리 확인

MAVSDK로 SITL에 연결해서 위치·자세 텔레메트리를 몇 개 받아 출력하는 가장 기본적인 스크립트.
포트는 `14540`(PX4가 컴패니언 컴퓨터용으로 여는 포트) 사용.

```bash
python test_connection.py
```

### 8-2. `arm_takeoff_land.py` — 고수준 명령 제어

`drone.action` API로 Arm → Takeoff(3.0m) → Hover 5초 → Land까지 완전히 코드로 실행.
QGC 없이도 콘솔 로그만으로 전체 비행 사이클 확인 가능.

```bash
python arm_takeoff_land.py
```

### 8-3. `offboard_velocity_test.py` — Offboard 저수준 속도 제어

`drone.offboard` API로 velocity setpoint를 직접 스트리밍하는 실습.
전진(velocity body frame), 회전(yaw rate) 명령 모두 안정적으로 작동 확인됨.

핵심 패턴:
- Offboard 진입 전 setpoint 최소 1회 선행 전송 필수
- 진입 후 0.5초 이내 주기로 계속 스트리밍 필요 (안 하면 PX4가 자동으로 Offboard 종료)
- yaw 텔레메트리는 필요할 때만 `anext()` 호출하지 말고, 백그라운드 태스크로 계속 소비하면서
  최신값만 변수에 저장하는 방식 사용할 것 (스트림 적체 방지)
- `action.takeoff()` 후 고정 시간 sleep으로 넘어가지 말고, `telemetry.position()`으로 실제
  고도(`relative_altitude_m`)가 안전 고도(예: 1.5m 이상)에 도달했는지 확인 후 Offboard로 전환할 것
  (목표 고도 미도달 상태에서 Offboard의 `vz=0`이 낮은 고도를 그대로 고정시켜버리는 문제 있었음)

```bash
python offboard_velocity_test.py
```

### 8-4. `yaw_rate_sweep_test.py` — yaw rate 이상 현상 진단용

여러 yaw_rate(10/20/45/60/90 deg/s)를 한 비행 세션에서 순차 테스트하고 CSV로 기록.
`flight_mode` 실시간 로깅 포함 (Offboard 이탈 여부 확인용).

```bash
python yaw_rate_sweep_test.py
```

실행 후 `yaw_sweep_YYYYMMDD_HHMMSS.csv` 파일 생성됨.

### 8-5. `wind_disturbance_baseline.py` — 바람 외란 PID-only 베이스라인 (단일 조건)

`gz_x500_windy` 월드(고정 바람, `linear_velocity=(5,2,0)` m/s)에서 PINN 보정 없이 순수
PID로만 비행하며 얼마나 밀리는지 기록. 호버링(위치 고정, 30초) → 직선비행(vx=2m/s, 20초)
두 phase.

핵심 결과: 강풍에서도 위치오차는 PID가 알아서 5cm 이내로 잘 잡음. 대신 그 대가로
roll/pitch가 바람 세기에 비례해서 계속 기울어진 채로 유지됨 (예: 8.5m/s에서
roll≈3°, pitch≈5° 정도). 이 "자세 바이어스"가 외란의 흔적이자 나중에 PINN이 추정할 대상.

```bash
python wind_disturbance_baseline.py
```

### 8-6. `wind_sweep_baseline.py` — 바람 조건 스윕 + 반복 (calm/light/default/strong/crosswind)

한 번의 비행 세션 안에서 `gz topic -t /world/windy/wind -m gz.msgs.Wind -p '...'`로
Gazebo의 바람을 **런타임에 실시간으로 바꿔가며**(SITL 재시작 불필요) 5개 조건 × 3회 반복
호버링 + 1회 직선비행을 수집. `set_wind()` 헬퍼 함수가 이후 스크립트에서도 재사용됨.

결과: 바람이 강할수록 roll/pitch 기울임이 단조 증가, crosswind(순수 옆바람)는 pitch
대신 roll 쪽으로 쏠리는 등 물리적으로 타당한 패턴 확인.

```bash
python wind_sweep_baseline.py
```

### 8-7. `wind_random_sweep.py` — PINN 학습용 무작위 바람 데이터 수집 (고정바람)

`wind_sweep_baseline.py`(고정 5조건)보다 훨씬 다양하게: 풍속 0~10m/s, 방향 0~360°를
무작위로 40개 뽑아서(고정 시드=42, 재현 가능) 각 8초씩 호버링하며 상태(속도/자세)를
20Hz로 기록. `gz topic`으로 직접 설정한 값이라 **정답 바람벡터를 100% 정확히 앎** →
지도학습 라벨로 그대로 사용 가능. 결과는 `../logs/wind_random_TIMESTAMP.csv`.

`--n`, `--speed-min`, `--speed-max`, `--seed` 옵션으로 특정 구간(예: 강풍 위주)만
집중해서 추가로 모을 수 있음.

```bash
python wind_random_sweep.py
python wind_random_sweep.py --n 35 --speed-min 4 --speed-max 11 --seed 77   # 강풍 위주 추가 수집 예시
```

### 8-7-1. `wind_gust_sweep.py` — PINN 학습용 gust(시간에 따라 변하는 바람) 데이터 수집

`wind_random_sweep.py`는 한 트라이얼 내내 바람이 고정이었음. 이건 에피소드마다
`vx(t)=base+amp*sin(2πt/period+phase)` 형태로 바람이 계속 변하도록 해서, "지금 이
순간의 바람을 실시간 추적"하는 걸 배우게 하는 데이터. 기본 15개 에피소드(20초씩).
Gazebo에는 `GUST_UPDATE_INTERVAL_S`(1.0초) 간격으로만 갱신해서 보내지만(계단식 근사,
너무 자주 호출하면 `gz topic pub` 프로세스 spawn 비용 때문에 확 느려짐 - 처음 0.25초로
했다가 26분 걸려서 1.0초로 늘림), CSV 라벨은 매 로그 순간의 정확한 연속함수 값을 씀.
`wind_random_sweep.py`와 동일하게 `--n`/`--speed-min`/`--speed-max`/`--seed` 지원.

```bash
python wind_gust_sweep.py
python wind_gust_sweep.py --n 12 --speed-min 4 --speed-max 10 --seed 456   # 강풍 gust 추가 수집 예시
```

### 8-8. `pinn_correction_interface_test.py` — 보정 배관(pipeline) 연결 테스트

아직 실제 PINN 없이, "상태 읽기 → 보정값 계산(`compute_correction()`) → setpoint에
더하기 → 실제 송신"이라는 배관이 끝까지 연결되어 있는지 검증하는 스크립트. 알려진
테스트값(north+1.5m, east-1.0m)을 15초 뒤에 주입해서, 드론이 실제로 그만큼 이동하는지
확인 (로그가 아니라 실제 비행 결과로 검증). 무풍/실제 강풍(8.5m/s) 양쪽에서 검증 완료.

```bash
python pinn_correction_interface_test.py
```

### 8-9. `pinn_wind_correction_test.py` — 학습된 PINN을 실제로 연결 (강풍 1개 조건 A/B)

`offline_training/wind_estimator.pt`를 로드해서 실시간 추론 → 보정 → 송신까지 전부
연결한 최종 버전. "보정 OFF(PID-only)" vs "보정 ON"을 같은 강풍(8m/s,3m/s) 온셋
상황에서 비교. **가속도 피드포워드 방식**(자세한 설계 이유는 12절 참고)을 사용하며,
결과: 바람 온셋 시 피크 위치오차가 0.469m → 0.212m로 약 55% 감소.

```bash
python pinn_wind_correction_test.py
```

### 8-10. `pinn_wind_correction_sweep.py` — PINN 보정 다중 조건 A/B 스윕

8-9의 다중 조건 버전. calm/light/default/strong/crosswind 5개 조건 각각에서 OFF→ON
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
# pinn_train 환경의 python 사용 (7-1절 참고)
python train_wind_estimator.py ../logs/wind_random_TIMESTAMP.csv
python train_wind_estimator.py ../logs/wind_random_*.csv ../logs/wind_gust_*.csv --kfold 5
```

---

## 9. (해결됨) Offboard yaw rate 비결정적 동작

**증상**: `VelocityBodyYawspeed`로 일정한 yaw 각속도를 지속 명령해도, 실제 회전이
멈췄다 움직였다를 예측 불가능하게 반복함.

**원인**: `action.takeoff()` 후 고정 5초 sleep만으로 Offboard에 진입하던 구조라, 목표 고도
도달 전에 Offboard의 `vz=0`이 낮은 고도(수 cm)를 그대로 고정시켜버렸고, 지면 근처에서는
yaw 회전이 실행되지 않았음. **해결**: 고정 시간 대기 대신 `telemetry.position()`으로 실제
안전 고도 도달을 확인한 후 Offboard로 전환하도록 수정 (8-3절 반영됨).

---

## 10. 트러블슈팅 모음

| 증상 | 원인/해결 |
|---|---|
| `ninja: error: unknown target 'list_vmd_make_targets'` | 첫 빌드 전 타겟 조회 명령을 실행해서 발생. `make distclean` 후 재시도하거나 무시하고 바로 빌드 |
| QGC가 "Disconnected"로 안 붙음 | WSL2와 Windows 네트워크 분리 문제. `.wslconfig`에 `networkingMode=mirrored` 설정 후 `wsl --shutdown` |
| MAVSDK `udp://` deprecated 경고 | `udpin://` 또는 `udpout://`로 명시 (`udpin://0.0.0.0:14540` 형태 권장) |
| yaw 텔레메트리 값이 실시간 반영 안 됨(계속 같은 값) | `anext()`를 필요할 때만 호출하는 방식의 스트림 적체 문제. 백그라운드 태스크로 계속 소비하며 최신값만 저장하는 방식으로 변경 |
| Offboard 회전 명령이 불규칙하게 동작 | (해결됨) 9절 참고 — takeoff 후 고정 sleep 대신 실제 고도 확인 후 Offboard 진입 |
| PINN 보정 켜니 위치오차가 계속 커지다 발산(roll/pitch 수십도까지) | (해결됨) 12-3절 참고 — 모델 입력(pos_err)이 보정 자신의 영향을 다시 입력받는 폐루프였음. position을 흔드는 방식 자체를 버리고 가속도 피드포워드로 구조 변경 |

---

## 11. 다음 단계

**완료**:
- 바람 외란(`gz_x500_windy`) 환경에서 PID-only 베이스라인 데이터 수집 (8-5, 8-6절)
- PINN 바람 추정 모델 학습 + 실제 비행 연결 + 다중 조건 검증 (12절)
- calm(무풍)에서 보정이 미세하게 손해보던 문제 → `WIND_DEADBAND_MPS` 추가로 해결,
  5개 조건 전부 개선(+8.2%~+71.3%)으로 재검증 완료 (12-5절)
- PINN 학습 전용 conda 환경(`pinn_train`) 분리 (7-1절)
- gust(시간에 따라 변하는 바람) 데이터 확장 시도 + k-fold 교차검증 도입 (12-6절) —
  결론: gust를 포함하면 풍속 추정 오차가 0.37~0.5m/s → 1.8m/s로 뚜렷이 나빠짐.
  모델 크기/정규화/데이터량 여러 방향 시도했지만 못 뚫음. **현재 구조(작은 MLP +
  최근 1초 히스토리)의 한계로 잠정 결론**

**남은 것 (후보, 우선순위 미정)**:
- gust 정확도 개선하려면 모델 구조를 아예 바꾸거나(RNN/시계열류) 데이터를 지금보다
  훨씬 큰 규모로 모아야 할 것으로 보임 (12-6절 결론 참고) — 우선순위 낮음으로 보류 중
- `ACCEL_GAIN`(현재 0.15), `WIND_DEADBAND_MPS`(현재 1.0) 둘 다 손으로 정한 값이라
  좀 더 체계적으로 튜닝
- gust 보정 A/B 테스트(pinn_wind_correction_sweep.py를 gust 조건으로) — 지금까지
  A/B 검증은 전부 고정바람 조건에서만 했음
- MAVLink 브릿지 ↔ PINN 추론 연동을 실제 하드웨어(Jetson + 컴패니언 컴퓨터) 기준으로
  재설계 — 지금까지는 SITL 위에서 같은 Python 프로세스 안에 다 넣어서 돌린 것
- 캡스톤 보고서용으로 결과 정리/시각화
- (교훈) 학습 스크립트처럼 CPU를 많이 쓰는 작업은 SITL 비행 테스트와 동시에 돌리지
  말 것 — 한 번 IMU 타임스탬프 에러가 난 적 있음 (다행히 드론엔 문제 없었음)

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
  > y=East) 라서 물리 잔차 계산 시 축을 바꿔줘야 함 (`train_wind_estimator.py`의
  > `physics_residual()` 주석 참고).
- **학습 결과** (미학습 바람조건 8개로 검증): 풍속(speed) MAE **0.37~0.50 m/s**
  (평균 실제 풍속 4.6m/s 대비). 과적합 방지를 위해 마지막 epoch이 아니라 validation
  MAE가 가장 낮았던 시점의 체크포인트를 저장하도록 함.
- **모델 파일**: `offline_training/wind_estimator.pt` (가중치 + 입력 정규화 통계
  `X_mean`/`X_std` + `window`/`features` 설정까지 같이 저장돼서, 추론 스크립트가
  그대로 로드해서 씀).

### 12-3. 1차 시도(실패): position setpoint 보정 → 폐루프 발산

**설계**: 추정 바람에 `-GAIN`을 곱해서 position setpoint(목표 위치)에 더함
("바람 반대방향으로 목표를 살짝 밀어서 PID가 더 일찍 반응하게 유도").

**결과**: 처음엔 roll이 -18.6°, 위치오차 18.6m까지 발산. `pos_err`를 계산하는 기준을
고쳤더니(그래도 여전히 발산, 심지어 더 심하게: 위치오차 52m, pitch 39.7°) 더 근본적인
문제가 있다는 걸 알게 됨.

**근본 원인**: 학습 데이터에서 `pos_err`(위치오차)는 항상 "순수하게 바람 때문에 생긴
오차"였음 (학습 시엔 보정이 없어서 setpoint가 고정이었으니까). 그런데 보정을 켜면
`pos_err`의 일부가 **"보정 자신이 setpoint를 계속 움직여서 생긴 추종 지연"**이 됨.
PID는 유한한 응답속도를 가지므로, setpoint가 계속 움직이면 어쩔 수 없이 뒤처지는데,
모델은 이 커진 `pos_err`를 "바람이 더 세졌다"고 오인 → 보정을 더 키움 → 지연이 더
커짐 → ... 양의 피드백. 학습 데이터엔 `pos_err`가 이렇게 큰 경우(1~10m, 학습땐 항상
<0.5m)가 없어서, 모델이 완전히 학습 범위 밖에서 예측하며 신경망이 그 영역에서
아무렇게나 행동한 것.

더 근본적으로는: 이전 `wind_disturbance_baseline.py` 실험에서 이미 PID가 강풍에서도
**자세(roll/pitch) 트림만으로 정상상태 위치오차를 5cm 이내로 거의 완벽히** 없애고
있다는 걸 확인했었음. 즉 "정상상태 위치오차"는 애초에 고칠 여지가 별로 없는 상태였고,
position setpoint를 흔드는 보정 방식 자체가 이 문제에 안 맞는 지렛대였음.

**시도한 완화책들 (순서대로, 각각 부분적으로만 도움됨)**:
1. `pos_err` feature 제거 + 재학습 → 오히려 검증 정확도도 개선됨 (val MAE 0.49→0.37m/s,
   pos_err가 원래 노이즈만 많던 feature였던 것)
2. 보정량에 안전 클램핑(±3m) 추가 → 무한 발산은 막았지만 여전히 baseline보다 나쁨
   (피크오차 3.9m)
3. 추정치에 EMA 스무딩(시정수 0.4초) 추가 → 노이즈로 인한 setpoint 흔들림 완화, 그래도
   여전히 baseline보다 나쁨 (피크오차 2.5m)

세 가지를 다 적용해도 baseline(0.43m)을 못 넘어서, **position 보정 방식 자체를 포기**.

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

지금까지(12-2절)는 한 트라이얼 내내 바람이 고정이었음. 실제 바람은 계속 변하므로,
`wind_gust_sweep.py`로 사인파 형태로 변하는 바람 데이터(15개 에피소드, 주기 4~10초)를
추가 수집해서 기존 고정바람 데이터와 합쳐 재학습을 시도함.

**시행착오 순서**:
1. gust 데이터 합쳐서 그대로 재학습 → val_MAE 0.37~0.5 → **1.82 m/s**로 크게 악화.
   원래 쉬웠던 고정바람 단독 평가도 0.82로 같이 나빠짐 (gust=원래 더 어려운 문제인
   것도 있지만, 두 데이터를 섞은 학습 자체도 영향을 준 것으로 보임)
2. **모델을 키워봄**(hidden 64→128, window 10→20, 풀배치→미니배치) → train_loss는
   거의 0까지 떨어지는데 val은 전혀 안 좋아짐. **전형적인 과적합** (best_epoch=4,
   사실상 학습 초반이 제일 나았음) → 데이터 양(조건 55개)에 비해 모델이 이미
   충분하다 못해 넘쳤다는 신호. 모델을 더 키우는 방향은 기각.
3. **weight decay(L2) 추가 + hidden 64로 원복** → 여전히 큰 개선 없음 (val_MAE
   1.8~2.0대에서 거의 안 움직임)
4. **조건별로 오차를 까봄** → 딱 하나(강풍 9.89m/s 조건)가 MAE 6.7로 폭발하며 평균을
   왜곡시키고 있었음. 나머지는 대부분 0.3~1.6 수준으로 준수. → "강풍 데이터가
   부족해서 그런가?" 가설 세움
5. **강풍(4~11m/s) 위주로 데이터 추가 수집** (`wind_random_sweep.py --speed-min 4
   --speed-max 11`, `wind_gust_sweep.py --speed-min 4 --speed-max 10`) → 절대오차는
   더 커짐(2.26)but 평균풍속도 같이 올라가서(6.96) 상대오차는 오히려 개선(38%→32.5%).
   그런데 이번엔 나쁜 조건이 하나가 아니라 여러 개로 늘어남
6. **풍속-오차 상관관계를 직접 계산**: 상관계수 0.578로 애매함, 게다가 저풍속(0~2m/s)
   구간도 상대오차 74%로 이미 나쁨 → "고풍속만 어렵다"로 깔끔히 설명 안 됨
7. 여기서 중요한 걸 깨달음: **조건이 100개 안팎일 때 한 번의 무작위 80/20 분할만으로
   평가하면, 그 분할에 뭐가 뽑히느냐에 따라 결과가 크게 흔들림** (재학습마다 "나쁜
   조건"이 매번 다르게 나왔던 게 이 때문). 모델/데이터를 바꾼 효과인지 그냥 평가가
   불안정한 건지 구분이 안 되고 있었음

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
