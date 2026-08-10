# 프로젝트 구조

지금까지 구현한 파이프라인/폴더/파일 간 관계를 한눈에 보기 위한 문서입니다.
실행 방법은 [USAGE.md](USAGE.md), 설계 이유·실험 결과는 [EXPERIMENTS.md](EXPERIMENTS.md),
개요·결과 요약은 [README.md](README.md) 참고.

---

## 1. 전체 파이프라인 (데이터 흐름)

```
[1. 데이터 수집]                          [2. 오프라인 학습]                    [3. 실비행 제어 연결]
sim_scripts/data_collection/          offline_training/                  sim_scripts/correction_experiments/

wind_random_sweep.py  ──┐
                         ├──▶ logs/*.csv ──▶ train_wind_estimator.py ──▶ wind_estimator.pt ──┐
wind_gust_sweep.py    ──┘         │           (wind_pinn_model.py 정의 사용)                  │
                                   │                                                          ├─▶ pinn_wind_correction_sweep.py
                    (plot_session_grid.py /                                                   ├─▶ pinn_wind_correction_gust_sweep.py
                     plot_combined_summary.py로                                                └─▶ pinn_correction_param_tuning.py
                     PNG 시각화)                                                                        │
                                                                                                          ▼
wind_yaw_generalization_test.py ──▶ logs/*.csv ──▶ evaluate_checkpoint.py (일반화 검증)         PX4 SITL(내장 PID) 제어
                                                                                                          │
                                                                                                          ▼
                                                                                        logs/pinn_correction_*.csv (A/B 결과)
```

`run_yaw_collection_sessions.sh`는 `wind_random_sweep.py`를 여러 세션으로 나눠
반복 실행하는 오케스트레이션 스크립트, `watch_collection.sh`는 그 워치독입니다.

---

## 2. 제어 루프 구조 (correction_experiments 실행 시 실시간 동작)

```
[PX4 SITL(Gazebo)] ──(텔레메트리: 속도·자세)──▶ [MAVSDK 수신]
                                                       │
                                                       ▼
                                     최근 WINDOW(20스텝, 1.0초) 상태 버퍼
                                                       │
                                                       ▼
                              WindPINN(state) ──▶ 추정 바람벡터 (wind_n, wind_e)
                                                       │
                                  accel = -ACCEL_GAIN * wind  (+ deadband 적용)
                                                       │
                                                       ▼
                MAVSDK set_position_velocity_acceleration_ned(position=고정값, accel=위 값)
                                                       │
                                                       ▼
                            [PX4 내장 PID] 셋포인트 추종 — 실제 모터 제어는 전부 여기서
                                                       │
                                                       ▼
                                         [Gazebo 물리 시뮬레이션 (바람 외란 포함)]
                                                       │
                                       (다시 텔레메트리로 피드백, 루프 반복)
```

**중요**: PINN은 컨트롤러가 아니라 "셋포인트에 추가로 얹는 가속도 힌트"만 계산합니다.
실제 위치/자세를 잡는 제어(PID)는 전부 PX4 내부에서 이뤄지며, 이 프로젝트가 만든
코드는 그 PID를 대체하지 않습니다.

---

## 3. 폴더/파일 구조

```
quad-pinn-project/
├── README.md                프로젝트 개요, 결과 요약, 다음 할 일
├── EXPERIMENTS.md            실험 상세 기록 (설계 이유, 실패 사례, 근거 있는 결론)
├── USAGE.md                  파일별 실행 방법
├── setup_guide.md            환경 구축 + 트러블슈팅
├── STRUCTURE.md               (이 문서) 프로젝트 구조/파이프라인 개요
│
├── sim_scripts/
│   ├── data_collection/       PINN 학습용 데이터 수집
│   │   ├── wind_random_sweep.py             고정바람, yaw 그리드로 라벨링
│   │   ├── wind_gust_sweep.py               변동(gust)바람, yaw 그리드로 라벨링
│   │   ├── wind_yaw_generalization_test.py  학습 밖 yaw에서 검증용 소량 수집
│   │   ├── run_yaw_collection_sessions.sh   대규모 수집 오케스트레이션(세션 분할)
│   │   ├── watch_collection.sh              위 스크립트가 멈췄는지 감시
│   │   ├── plot_session_grid.py             세션 1개 → yaw별 격자 PNG
│   │   └── plot_combined_summary.py         여러 세션 → 모자이크/겹침 PNG
│   │
│   └── correction_experiments/  학습된 PINN을 실비행 제어에 연결·검증
│       ├── pinn_wind_correction_sweep.py        고정바람 5조건 OFF/ON A/B
│       ├── pinn_wind_correction_gust_sweep.py   gust 4조건 OFF/ON A/B
│       └── pinn_correction_param_tuning.py      ACCEL_GAIN/DEADBAND 스윕
│
├── offline_training/            PINN 학습 (지도학습 + 물리 손실)
│   ├── wind_pinn_model.py       물리방정식·모델구조·하이퍼파라미터 정의 (import 전용)
│   ├── train_wind_estimator.py  학습 실행 (--kfold 교차검증 지원)
│   ├── evaluate_checkpoint.py   저장된 모델을 새 CSV로 평가
│   └── wind_estimator.pt        학습 산출물 (가중치 + 정규화통계 + 설정, git 추적)
│
├── logs/                         실행 결과 CSV (스크립트별 파일명 접두사로 구분)
└── figures/                      수집/검증 결과 요약 PNG
```

---

## 4. 모델 구조 요약 (`WindPINN`, `offline_training/wind_pinn_model.py`)

```
입력 (window=20 × feature=6 = 120차원, flatten)
  vn, ve                                    관성좌표(NED) 속도
  roll_cos_yaw, roll_sin_yaw,
  pitch_cos_yaw, pitch_sin_yaw              yaw로 분해한 기체 자세각 (yaw_decompose)
        │
        ▼
Linear(120→64) → ReLU → Linear(64→64) → ReLU → Linear(64→2)
        │
        ▼
출력: [wind_vx_enu, wind_vy_enu]  (Gazebo world 표기)

+ 학습 가능 파라미터: k = exp(log_k)   (항력계수 묶음항, 초기값 exp(-2.0))
```

**손실함수**:

```
L = L_data + LAMBDA_PHYSICS(0.05) * L_physics

L_data     = MSE(모델 출력, 실측 바람벡터 라벨)
L_physics  = MSE(k*|v_rel|*v_rel,  유한차분으로 구한 실측 가속도)
             (v_rel = 추정바람 − 실측드론속도, ENU↔NED 축변환 포함)
```

자동미분(AD)으로 도함수를 구하는 과정은 없음 — `L_physics`의 "가속도"는 신경망
출력의 미분이 아니라 **측정 속도의 유한차분**이라, PINN 세미나(발표자료)가 정의한
콜로케이션+AD 기반 물리손실과는 계산 방식이 다름. 자세한 비교는
[EXPERIMENTS.md 12-15절](EXPERIMENTS.md#12-15-ramp-net-논문과-비교-검토-향후-loss-설계-방향) 참고.

---

## 5. 지금 구조의 스코프 (요약)

- **지배방정식**: 병진 항력식 `a = k|v_rel|v_rel` (회전/토크 방정식 아님)
- **컨트롤러**: PX4 내장 PID + MAVSDK 오프보드 가속도 피드포워드 (MPC 없음)
- **PINN 역할**: 바람벡터 추정(보조 신호) — 동역학 모델 자체를 대체하진 않음
- **검증 데이터**: PX4 SITL(Gazebo) 시뮬레이션만 — 지그 실측/실비행 데이터 없음

이 스코프를 벗어나는 방향(토크 기반 전환, MPC 직접 구현, RAMP-Net식 AD 콜로케이션
손실 등)은 EXPERIMENTS.md 12-15절에 후보로 정리돼 있습니다.
