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
    │   ├── data_collection/               # PINN 학습용 데이터 수집 (고정바람/더더링/gust/전환구간 + 오케스트레이션, 상세는 USAGE.md)
    │   └── correction_experiments/        # PINN 보정 A/B·파라미터 튜닝·회전 피드포워드 (상세는 USAGE.md)
    ├── offline_training/                  # PINN 학습 파이프라인
    │   ├── wind_pinn_model.py
    │   ├── train_wind_estimator.py
    │   ├── evaluate_checkpoint.py
    │   └── wind_estimator.pt             # 학습된 모델 체크포인트
    ├── logs/                              # 실험 결과 CSV (wind_random_*, wind_gust_*, pinn_correction_*)
    └── figures/                           # 수집 실행별(시간+조건명) 폴더에 정리된 요약 PNG
        └── wind_random_TIMESTAMP_n{N}x{M}_td{S}/   # 세션별 격자 PNG + 모자이크 + 전체 겹침
```

> 참고: 초기 스크립트들은 결과 CSV를 `sim_scripts/` 안에 그냥 저장하도록 되어 있었음.
> `wind_random_sweep.py`부터는 `../../logs/`(지금은 `sim_scripts/`의 하위 폴더에서
> 실행하므로 두 단계 위)에 저장하도록 통일함 - 앞으로 작성하는 스크립트도 이 규칙을 따를 것.
>
> 초기 검증용 스크립트들(`test_connection.py`, `arm_takeoff_land.py`,
> `offboard_velocity_test.py`, `yaw_rate_sweep_test.py`, `wind_disturbance_baseline.py`,
> `wind_sweep_baseline.py`, `pinn_correction_interface_test.py`,
> `pinn_wind_correction_test.py`)은 목적을 다 마치고(버그 재현/배관 검증/물리 검증 등,
> USAGE.md·아래 8절 참고) 제거함 - 필요하면 git 이력에서 복원 가능.
>
> `runtime/`(Jetson 실시간 파이프라인)은 아직 폴더도 안 만든 상태 - EXPERIMENTS.md
> "다음 단계" 참고.

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

## 8. 트러블슈팅 모음

| 증상 | 원인/해결 |
|---|---|
| `ninja: error: unknown target 'list_vmd_make_targets'` | 첫 빌드 전 타겟 조회 명령을 실행해서 발생. `make distclean` 후 재시도하거나 무시하고 바로 빌드 |
| QGC가 "Disconnected"로 안 붙음 | WSL2와 Windows 네트워크 분리 문제. `.wslconfig`에 `networkingMode=mirrored` 설정 후 `wsl --shutdown` |
| MAVSDK `udp://` deprecated 경고 | `udpin://` 또는 `udpout://`로 명시 (`udpin://0.0.0.0:14540` 형태 권장) |
| yaw 텔레메트리 값이 실시간 반영 안 됨(계속 같은 값) | `anext()`를 필요할 때만 호출하는 방식의 스트림 적체 문제. 백그라운드 태스크로 계속 소비하며 최신값만 저장하는 방식으로 변경 |
| 같은 조건인데 A/B 테스트 결과가 이전 실행과 크게 다름(특히 calm 베이스라인이 갑자기 커짐) | EXPERIMENTS.md 12-8절에서 **단 1회** 관측됨(SITL 65분 연속 실행 후). "장시간 실행이 원인"이라는 건 검증 안 된 가설이지 확립된 규칙이 아님 — 재현 실험은 안 해봤음. 다만 이상하게 튀는 결과가 나오면 SITL을 재시작해서 재현되는지 확인해볼 가치는 있음 |
| `make px4_sitl` 재구성 중 `kconfiglib is not installed` 에러, 심하면 `build/px4_sitl_default` 폴더 자체가 사라짐 | CMake가 Python3를 찾을 때 PATH상 `px4sim` conda 환경의 python을 잡는 경우가 있는데, 거기엔 PX4 빌드용 패키지(`kconfiglib` 등)가 없어서 재구성이 실패하며 빌드 폴더를 정리해버림. `environment-px4sim.yml`에 `PX4-Autopilot/Tools/setup/requirements.txt` 전체를 포함시켜 재발 방지함 (`conda env update -f environment-px4sim.yml`로 기존 환경에도 추가 가능) |
| `HEADLESS=1 make px4_sitl ...`를 백그라운드로 오래 돌리면 출력 파일이 수 GB까지 불어남 | PX4의 `pxh>` 콘솔이 진짜 터미널이 아닌 파이프로 연결되면 프롬프트를 계속 지우고 다시 그리는(ANSI escape) 동작을 무한 반복하는 것뿐 — SITL 자체는 정상 동작. 출력을 `> /dev/null 2>&1`로 버리고 백그라운드로 띄우면 문제없음 |
| 세션형 수집 스크립트가 원인 불명으로 계속 멈춤(연결은 되는데 GPS/홈 위치 확인에서 안 넘어감) — **"빌드 손상"이 아니라 `parameters.bson` 오염임에 유의** | 진짜 원인은 빌드(컴파일된 코드)가 아니라 `rootfs/parameters.bson`(런타임 설정 파일)이 손상되는 것 — 세션 사이 `pkill -9`로 강제종료를 반복하다 이 파일을 다 못 쓰고 죽는 게 원인. 손상되면 EKF2가 GPS를 영구히 안 믿는 상태(`ESTIMATOR_CONST_POS_MODE` 고착)에 빠져 GPS 락이 영영 안 됨. **`mv rootfs/parameters.bson{,.bak}` 후 재시작만으로 즉시 해결** — `make distclean` + 완전 재빌드(10~25분)는 이 파일이 빌드 폴더 안에 있어서 부수적으로 같이 지워지는 것뿐이지 실제로 필요한 조치가 아님(대조실험으로 확인됨 - PX4 소스를 되돌려도 재현됨). 그런데도 재빌드로 "해결"한 사례가 반복 기록돼 있는 건(12-16/30/34/36절) 매번 이 사실을 잊고 더 비싼 방법을 쓴 것 — **다음엔 먼저 parameters.bson부터 지울 것.** 예방책: `restart_sitl()`이 매 세션 재시작마다 예방적으로 삭제하도록 이미 반영해둠 (수동으로 `make px4_sitl`을 직접 돌릴 땐 이 예방 로직을 안 거치니 주의) |
| SITL을 직접 껐다 켜는 스크립트를 실수로 두 번 동시 실행 | SITL/포트 충돌. 재시작 전 `pgrep -af "<스크립트명>"`으로 이미 떠 있는지 확인 |
| MAVSDK 스크립트가 예외/트레이스백 후에도 프로세스가 안 죽음 | `asyncio.run()` 내부 grpc 채널 정리가 멈추는 경우 있음 — 데이터 수집 스크립트는 `os._exit()`로 이미 대응, 그 외는 `timeout -k 15 <Ns> python ...`로 OS 레벨 타임아웃 감쌀 것 |
| 강제 종료 후 다음 실행에서 `arm(): COMMAND_DENIED` | 죽은 `mavsdk_server` 좀비가 포트(50051)를 물고 있는 게 원인 — `pgrep -af "px4|gz sim|mavsdk_server"`로 정리 후 SITL 재시작 |
