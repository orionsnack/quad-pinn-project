# RAMP-Net 완전 재현 로드맵

**이 문서의 위치**: [EXPERIMENTS.md 12-15절](EXPERIMENTS.md#12-15-ramp-net-논문과-비교-검토-향후-loss-설계-방향)에서는
"손실함수 설계만 부분 차용"으로 스코프를 좁히기로 결정했음. 이 문서는 그 결정과
별개로, **완전 재현(지배방정식 확장 + MPC 직접 구현 + 베이스라인 4종)을 실제로
진행할 경우**의 실행 순서를 담는다. 즉 "지금 하기로 한 것"이 아니라 "하게 되면
이 순서로 한다"는 계획 문서. 개요/결과는 [README.md](README.md), 지금까지 실제로
한 것의 기록은 EXPERIMENTS.md 참고.

---

## 0. 배경 — 왜 처음부터 다 만들지 않는가

RAMP-Net(및 베이스라인인 GP-MPC, KNODE-MPC)은 전부 "명목 동역학 모델(nominal) +
학습된 잔차모델(residual)을 MPC 예측에 함께 쓴다"는 같은 구조의 변형이다. 이 구조를
이미 구현해 공개해 둔 저장소들이 있어서, 처음부터 다 짜는 대신 그걸 우리 스택
(PX4 SITL + Gazebo Harmonic `gz_x500_windy`, MAVSDK-Python)에 맞게 이식하는 쪽이
훨씬 빠르다. 아래는 실제로 clone하지 않고 README/구조/메타데이터(★, 라이선스,
최근 커밋)만 확인한 상태의 후보 목록 — 0단계에서 실제로 검증한다.

| 저장소 | 역할 | ★ | 라이선스 | 마지막 커밋 | 시뮬레이터 |
|---|---|---|---|---|---|
| [DISCOWER/px4-mpc](https://github.com/DISCOWER/px4-mpc) | MPC 제어루프 골격 (ACADOS, ROS2) | 206 | BSD-3 | 2026-07 | **PX4 SITL + `gz_x500` (우리와 동일)** |
| [TUM-AAS/ml-casadi](https://github.com/TUM-AAS/ml-casadi) | PyTorch 모델 → CasADi/ACADOS 심볼릭 브릿지 | 250 | MIT | 2023-11 | - |
| [TUM-AAS/neural-mpc](https://github.com/TUM-AAS/neural-mpc) | "명목모델+MLP 잔차"를 ACADOS에 꽂는 참고 구현 | 275 | GPL-3.0 | 2025-03 | Gazebo Classic + RotorS (다름) |
| [uzh-rpg/data_driven_mpc](https://github.com/uzh-rpg/data_driven_mpc) | GP-MPC 원본 구현 (그대로 베이스라인 후보) | 360 | GPL-3.0 | 2021-04 | Gazebo Classic + RotorS (다름) |
| [SaxionMechatronics/px4_offboard_lowlevel](https://github.com/SaxionMechatronics/px4_offboard_lowlevel) | PX4 저수준(자세/추력토크/모터) offboard 제어 예제 | 94 | BSD-3 | 2026-05 | PX4 SITL, ROS2 |

**중요한 비대칭**: `px4-mpc`/`px4_offboard_lowlevel`은 우리 시뮬레이터와 맞지만
ROS2 기반이라 통신 계층이 지금 프로젝트(순수 MAVSDK)와 다름. `neural-mpc`/
`data_driven_mpc`는 통신 계층 걱정은 없지만(둘 다 참고용으로 로직만 가져올 것)
시뮬레이터가 아예 다름(RotorS). 즉 "시뮬레이터 일치"와 "통신계층 일치"를 동시에
만족하는 저장소는 없음 — 0~1단계에서 이 트레이드오프를 실제로 판단해야 함.

---

## 1. 전체 단계 개요

| 단계 | 내용 | 기간 | 의존성 | 산출물 |
|---|---|---|---|---|
| 0 | 후보 저장소 실제 검증 | 3~5일 | - | go/no-go 판단 |
| 1 | ROS2 수용 여부 결정 | 0.5일 | 0 | 아키텍처 결정 |
| 2 | Nominal MPC 베이스라인 완성 | 1~2주 | 1 | `gz_x500_windy`에서 도는 MPC |
| 3 | 데이터 파이프라인 확장(모터명령 로깅) | 3~5일 (2와 병렬) | - | 확장된 수집 스크립트 |
| 4 | PINN 잔차모델 재설계 + ml-casadi 연결 | 1.5~2주 | 2, 3 | MPC에 꽂힌 PINN |
| 5 | 폐루프 안정화 | 1~3주(가변) | 4 | 5개 바람조건 안정 비행 |
| 6 | 나머지 베이스라인(GP-MPC, KNODE-MPC, PID) | 1~2주 (2~5와 일부 병렬) | 4의 "슬롯" 구조 | 4종 컨트롤러 |
| 7 | 평가 인프라(DTW/RMSE) | 3~5일 | 6 | 비교 결과표 |
| 8 | 문서화 | - | 7 | 보고서 |

이 문서는 **0단계만** 상세히 다룬다. 1단계 이후는 0단계 결과에 따라 갈리는
부분이 많아서, 0단계가 끝난 뒤 이 문서에 이어서 채워 넣는다.

---

## 2. 0단계 상세

### 목표
1~8단계를 시작하기 전에, "README만 보고 판단한 것"이 실제로 맞는지 최소 비용으로
검증한다. 여기서 걸리는 문제(설치 실패, 버전 충돌, 우리 월드에서 안 됨)를 먼저
찾아야 1단계의 아키텍처 결정을 근거 있게 내릴 수 있다.

### 0-1. 새 conda 환경 준비
지금 프로젝트는 이미 `px4sim`(MAVSDK 비행 제어)과 `pinn_train`(PINN 학습) 두
환경을 분리해서 씀(setup_guide.md 7-1절). ACADOS/CasADi/ROS2는 둘 중 어디에도
안 맞으므로 **세 번째 환경**(가칭 `mpc_dev`)이 필요할 가능성이 높음 — 0단계에서
실제로 필요한 패키지를 확인한 뒤 `environment-mpc_dev.yml`로 확정.

```bash
conda create -n mpc_dev python=3.10
conda activate mpc_dev
```

### 0-2. `ml-casadi` 최소 예제 검증 (제일 먼저, 제일 리스크가 큰 부분)
전체 계획에서 "PyTorch로 학습한 우리 PINN을 ACADOS 안에 넣을 수 있다"는 게
핵심 전제인데, 아직 실제로 확인 안 됨. 여기서 막히면 4단계 전체가 재설계 대상이라
가장 먼저 검증.

```bash
git clone https://github.com/TUM-AAS/ml-casadi.git
cd ml-casadi
pip install -e .
```

확인할 것:
- 지금 `offline_training/wind_estimator.pt`(또는 그 구조를 그대로 쓰는 더미 모델)를
  `ml-casadi`로 감싸서 CasADi `Function`으로 export가 되는지
- 이 함수가 ACADOS의 동역학 모델 자리(`model.f_expl_expr` 등)에 실제로 들어가서
  솔버가 코드생성(`ocp_solver`)까지 성공하는지 (여기서 실패하면 "우리 모델
  구조를 ml-casadi가 지원하는 형태로 바꿔야 하는지" 판단 필요)
- **체크포인트**: 여기서 실패하면 대안은 (a) 모델 구조를 ml-casadi가 지원하는
  형태로 단순화 (b) RAMP-Net처럼 직접 AD 콜로케이션 손실을 손으로 구현 두 가지뿐
  — 이건 이후 로드맵을 크게 바꾸는 결정이라 빨리 확인해야 함

### 0-3. ACADOS 설치
`px4-mpc`, `ml-casadi` 둘 다 ACADOS가 전제. C 라이브러리 빌드가 필요해서 지금까지
프로젝트에서 겪은 환경 문제(setup_guide.md 트러블슈팅 표)보다 설치 자체의 난이도가
높을 수 있음.

```bash
git clone https://github.com/acados/acados.git --recursive
cd acados && mkdir build && cd build
cmake -DACADOS_WITH_QPOASES=ON .. && make install -j4
pip install -e ../interfaces/acados_template
```

확인할 것: `LD_LIBRARY_PATH`, `ACADOS_SOURCE_DIR` 환경변수 설정 후 공식 예제
(`acados/examples/acados_python/getting_started`)가 실제로 풀리는지.

### 0-4. `px4-mpc`를 우리 `gz_x500` 월드에서 실행 검증
```bash
git clone https://github.com/DISCOWER/px4-mpc.git
```
README 기준 필요한 것: ROS2, `px4_msgs`, `micro-xrce-dds-agent`, `px4-offboard`.
지금 프로젝트는 ROS2를 전혀 안 쓰므로 이 부분 설치량이 상당함 — 설치 자체가
1단계 판단(ROS2 수용 여부)에 필요한 데이터가 됨(설치가 얼마나 오래/복잡한지가
곧 "받아들일 만한 비용인지"의 근거).

확인할 것:
- 지금 쓰는 `gz_x500`(바람 없는 기본 월드)에서 `px4-mpc`의 예제 궤적 추종이 되는지
- (되면) `gz_x500_windy` 월드에서도 별 문제 없이 붙는지 — 월드만 바꾸는 거라
  큰 문제는 없을 것으로 예상되지만 확인 필요
- ROS2 설치/빌드에 실제로 걸린 시간, 지금 프로젝트 conda 환경들과 충돌 여부

### 0-5. (참고용, 실행까지는 안 함) `data_driven_mpc`/`neural-mpc` 코드 구조만 훑기
이 둘은 RotorS 기반이라 우리 환경에서 실행은 안 될 가능성이 높음 — 굳이 설치까지
안 하고, GitHub에서 아래 두 가지만 눈으로 확인:
- `data_driven_mpc`의 GP 잔차모델이 ACADOS 모델의 어느 지점에 꽂히는지(파일:
  `ros_gp_mpc/src` 내부 구조) — 6단계에서 GP-MPC 베이스라인 이식할 때 그대로 참고
- `neural-mpc`가 `data_driven_mpc`의 GP 자리를 MLP로 바꾸면서 정확히 뭘 바꿨는지
  (diff 개념으로) — 4단계에서 우리 PINN을 꽂을 때 같은 패턴을 따라감

### 0-6. Go/No-Go 판단 기준
0단계 끝에 아래 4가지 질문에 답이 나와 있어야 1단계로 넘어감:
1. `ml-casadi`로 우리 PINN 구조를 ACADOS에 넣을 수 있는가? (0-2)
2. ACADOS 설치가 이 환경(WSL)에서 안정적으로 되는가? (0-3)
3. `px4-mpc`가 `gz_x500_windy`에서 실제로 도는가? (0-4)
4. ROS2 설치/유지 비용이 감당할 만한가, 아니면 ACADOS 모델 정의만 뽑아내고
   통신은 MAVSDK로 직접 짜는 게 나은가? (0-4 결과 기반)

이 네 가지가 확인되면 1단계(ROS2 수용 여부 최종 결정)로 넘어가고, 이 문서의
"1단계 이후" 섹션을 이어서 채운다.

---

## 3. 아직 정하지 않은 것 (0단계 이후 결정 예정)

- PINN 잔차모델의 정확한 출력 형태 — "속도 잔차"(neural-mpc 방식) vs "전체 상태
  천이" vs 지금처럼 "바람벡터" 유지 + 물리식으로 변환, 세 가지 중 선택 (4단계에서 결정)
- KNODE-MPC 잔차모델(Neural ODE) 구현에 쓸 라이브러리(`torchdiffeq` 등) 확정
- ~~ROS2 사용 여부~~ → 0-4 실측 결과로 사실상 결정됨(아래 4절). MPC 핵심 로직에
  ROS2가 필요 없다는 게 확인돼서, 1단계는 "받아들일지 말지 판단"에서 "MAVSDK
  글루코드를 어떻게 짤지"로 성격이 바뀜

---

## 4. 0단계 실행 결과 (실제 검증 완료)

**실험 코드 위치**: `~/MyProjects/rampnet_research/` (이 프로젝트 git 저장소 밖 —
`quad-pinn-project`와는 별도 디렉터리, git 추적 안 됨). `mpc_dev` conda 환경에
`casadi 3.7.2`, `torch 2.13.0+cu130`(GPU 없어도 CPU로 동작), `numpy 2.2.6`,
`ml_casadi`(editable install), `acados_template`(editable install) 설치됨.
환경변수는 `~/MyProjects/rampnet_research/env.sh`에 저장(`ACADOS_SOURCE_DIR`,
`LD_LIBRARY_PATH`) — 다음에 이 환경 쓸 때 `source`만 하면 됨.

### 0-2 결과: ml-casadi 호환성 — 통과 (실제 체크포인트로 검증)

`ml_casadi.torch.nn.MultiLayerPerceptron(120, 64, 2, 2, 'ReLU')`이 지금
`WindPINN.net`(`Linear(120→64)-ReLU-Linear(64→64)-ReLU-Linear(64→2)`)과 구조가
정확히 일치함을 확인. **실제 학습된 `offline_training/wind_estimator.pt`의
가중치**를 state_dict 키 이름만 매핑(`net.0/2/4` → `input_layer`/
`hidden_layers.0`/`output_layer`)해서 그대로 로드:

- PyTorch(WindPINN) vs PyTorch(ml_casadi MLP) 출력 오차: **0.0** (완전 일치)
- PyTorch vs CasADi symbolic Function 출력 오차: **2.2e-7** (부동소수점 오차 수준)

→ 지금 학습 파이프라인을 바꾸지 않고도 산출물(`wind_estimator.pt`)을 그대로
ACADOS에 넣을 수 있음이 토이 예제가 아니라 **실제 체크포인트로 확인됨**.
검증 스크립트: `~/MyProjects/rampnet_research/test_wind_pinn_ml_casadi.py`.

### 0-3 결과: ACADOS 설치 — 통과

`git clone --recursive` → `cmake` → `make install -j8`까지 별문제 없이 완료
(WSL, gcc 13.3, cmake 3.28, X64_INTEL_HASWELL 타겟 자동 감지). 공식
`getting_started/minimal_example_ocp.py`(진자 OCP)를 실행해 코드생성→컴파일→
풀이까지 전 과정 확인: 잔차가 10회 반복 만에 `3.86e-7`까지 수렴, 정상 종료
(그래프 창은 WSL이라 안 뜨는 게 정상 — `plt.show()`에서 블로킹하는 것뿐이라
`MPLBACKEND=Agg`로 우회).

### 0-4 결과: px4-mpc — 핵심 로직은 ROS2 완전히 불필요, 실측 통과

`px4-mpc/px4_mpc/px4_mpc/{models,controllers}/`에 `grep -rl "rclpy"`를 돌려본
결과 **ROS2는 `mpc_quadrotor.py`(PX4 통신 노드) 하나에만 몰려있고, 모델/컨트롤러
자체는 `acados_template`+`casadi`+`numpy`+`scipy`만 사용**. 저장소에 이미 있는
`test_multirotor_rate_closedloop.py`(ROS2 불필요, 순수 수치 폐루프 테스트)를
그대로 실행:

- 상태 10차원(위치3+속도3+쿼터니언4), 입력 4차원(추력1+각속도3)의 비선형
  쿼드로터 모델(중력+추력+쿼터니언 운동학, `skew_symmetric`/`q_to_rot_mat` 포함)
- SQP_RTI로 100스텝 폐루프 실행, **스텝당 계산시간 중앙값 1.33ms, 최대 2.29ms**
  → 500Hz(스텝당 2ms 예산) 실시간 루프에도 여유 있게 들어가는 속도
- **결론**: ROS2를 설치하지 않고도 `models/multirotor_rate_model.py` +
  `controllers/multirotor_rate_mpc.py` 두 파일만 그대로 가져와서 우리 MAVSDK
  코드에 연결하는 게 가능해 보임 — 1단계의 "ROS2 받아들일지" 결정이 사실상
  **"(B) MPC 코드만 뽑아서 MAVSDK로 직접 연결"** 쪽으로 기움. 다만 이건 아직
  실제 PX4 SITL(`gz_x500_windy`)에 연결해서 비행시켜본 건 아니고 수치 시뮬레이션
  (acados 자체 적분기)까지만 확인한 것 — 실제 SITL 연동 검증은 2단계 과제로 넘어감
- 이 모델의 입력이 "추력+각속도"(rate control)라는 점도 중요 — PX4가 이미
  자세제어 루프를 담당하고, MPC는 그 위(속도/자세 목표) 수준에서만 개입함.
  MAVSDK `Offboard` 플러그인에 `set_attitude_rate`(추력+각속도) 셋포인트 메서드가
  있는지가 다음에 확인할 것(2단계 착수 시 1순위 확인 사항)

### 0-5 결과: data_driven_mpc 구조 — GP-MPC 이식 가능성 확인

`quad_mpc/quad_3d.py`(명목 동역학) + `quad_mpc/quad_3d_optimizer.py`
(`Quad3DOptimizer`, `GPEnsemble` 잔차모델을 `use_gp` 플래그로 껐다 켬) +
`model_fitting/gp.py`로 역할이 깔끔히 분리돼 있음. 다만 GP 보정은 ml-casadi처럼
신경망을 심볼릭 그래프에 통째로 넣는 방식이 아니라, **최적화 시점에 선형화된
보정치를 얹는 방식**이라 4단계(PINN 삽입)와는 메커니즘이 다름 — "명목+잔차
분리"라는 큰 구조만 참고하고, 실제 GP 삽입 코드를 그대로 재사용하려면 이
차이를 감안해야 함.

### Go/No-Go 판단 (2-6절 질문에 대한 답)

| 질문 | 답 |
|---|---|
| ml-casadi로 우리 PINN을 ACADOS에 넣을 수 있는가? | **예** — 실제 체크포인트로 오차 2e-7 확인 |
| ACADOS 설치가 이 환경에서 안정적으로 되는가? | **예** — 클론부터 빌드까지 에러 없이 완료 |
| px4-mpc가 우리 스택에서 실제로 도는가? | **부분 확인** — 모델+컨트롤러 자체는 검증(1.33ms/step), SITL 연동은 미검증 |
| ROS2 수용 비용이 감당할 만한가? | **질문 자체가 무의미해짐** — 핵심 로직에 ROS2가 안 필요해서 안 받아들여도 됨 |

**종합 결론: 4개 항목 모두 "진행 가능" 쪽으로 확인됨.** 1단계(원래 "ROS2 받아들일지
결정")는 완료된 것과 다름없고, 2단계(Nominal MPC 베이스라인 완성)로 바로 넘어갈
수 있는 상태. 2단계 착수 시 제일 먼저 할 일: (1) MAVSDK `Offboard`에
attitude-rate 셋포인트 메서드 존재 확인, (2) `models/`+`controllers/` 두 파일을
복사해와 우리 프로젝트 구조(`mpc_controller/` 신설 등)에 맞게 옮기기, (3) 실제
`gz_x500_windy` SITL에 연결해서 무풍 상태로 첫 폐루프 비행 시도.
