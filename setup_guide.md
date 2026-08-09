# PX4 SITL + Gazebo + MAVSDK 시뮬레이션 환경 구축 가이드

**작성 목적**: 다른 컴퓨터에서도 동일한 시뮬레이션 개발 환경을 그대로 재현하기 위한 기록

**프로젝트**: PINN 기반 쿼드콥터 외란 추정 및 제어 - 캡스톤 설계

**검증 환경**: Windows 11 + WSL2(Ubuntu 22.04) + PX4-Autopilot + Gazebo(Harmonic) + MAVSDK-Python 3.17.2

---

## 0. 사전 준비물

- Windows 11 (WSLg를 통한 GUI 지원, WSL2 미러 네트워킹 모드 사용을 위해 필요)
- WSL2 활성화 및 Ubuntu 22.04 배포판
- 넉넉한 디스크 공간 (PX4 + 서브모듈 기준 최소 10GB 이상 권장)

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

**가장 빠른 방법**: 이 프로젝트는 `environment-px4sim.yml` / `environment-pinn_train.yml`
두 파일로 conda 환경을 그대로 재현할 수 있음 (아래 7-1절 참고):

```bash
cd ~/MyProjects/quad-pinn-project
conda env create -f environment-px4sim.yml
conda env create -f environment-pinn_train.yml
```

두 환경이 왜 나뉘어 있는지, 각각 뭘 쓰는지는 7-1절에 설명. 아래는 yml 없이 손으로
따라 할 경우의 단계별 명령어(= yml 파일 안에 들어있는 내용과 동일).

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
    ├── README.md                          # 프로젝트 요약 + 스크립트 설명 + 실험 상세 기록
    ├── setup_guide.md                     # 환경 재현 가이드 (이 문서)
    ├── environment-px4sim.yml             # px4sim conda 환경 재현용 (7-1절)
    ├── environment-pinn_train.yml         # pinn_train conda 환경 재현용 (7-1절)
    ├── sim_scripts/                       # MAVSDK 테스트/실험 스크립트 (README 8절 참고)
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
> `runtime/`(Jetson 실시간 파이프라인)은 아직 폴더도 안 만든 상태 - README "다음 단계" 참고.

### 7-1. PINN 학습/추론용 패키지 (conda 환경 두 개를 씀)

- **`px4sim` 환경** (시뮬레이션 + 실시간 추론용): `mavsdk` 외에 실시간으로 학습된 모델을
  돌리기 위해 CPU 버전 PyTorch도 추가 설치함. `environment-px4sim.yml`로 재현 가능.
  ```bash
  conda env create -f environment-px4sim.yml
  # 직접 실행 시:
  conda activate px4sim
  pip install --index-url https://download.pytorch.org/whl/cpu torch
  pip install numpy pandas
  ```
- **오프라인 학습(`offline_training/train_wind_estimator.py`)용으로 `pinn_train`이라는
  이 프로젝트 전용 환경을 따로 만듦.** (처음엔 `text2cad`라는 다른 프로젝트 환경을 급하게
  빌려썼었는데, 남의 환경이라 나중에 그쪽이 바뀌면 같이 깨질 위험이 있어서 분리함)
  `environment-pinn_train.yml`로 재현 가능.
  ```bash
  conda env create -f environment-pinn_train.yml
  # 직접 실행 시:
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

> 두 yml 파일 다 conda 환경 자체(파이썬 버전 + PyPI 패키지)만 재현함. PX4 SITL,
> Gazebo, QGroundControl 같은 시스템 레벨 설치는 위 1~6절을 그대로 따라야 함 —
> yml로 대신할 수 없는 부분.
