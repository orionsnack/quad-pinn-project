"""
PINN 외란 추정 모델의 "물리/모델 정의" 부분만 모아둔 파일.
방정식이나 하이퍼파라미터를 고치고 싶으면 여기만 보면 됨 - 데이터 로딩/학습 루프
(train_wind_estimator.py)나 실제 비행 제어(pinn_wind_correction_*.py)와 분리해뒀음.

물리 제약(PINN residual) 두 갈래:

1) 병진 (기존, 변경 없음) - 항력 방정식
     v_rel = wind_pred(NED) - v_drone(NED)
     a_drag_pred = k * |v_rel| * v_rel      (k = 0.5*rho*Cd*A/m, 학습 가능한 스칼라)
     a_measured  = 유한차분(v_drone, dt)
     physics_loss_trans = MSE(a_drag_pred, a_measured)

2) 회전 (신규, 2026-08-11 추가) - 회전운동방정식(세미나 자료 "졸업설계 주제" 방정식)
     J * omega_dot = tau_motor(물리로 직접 계산) + tau_disturbance(PINN이 추정)
     tau_motor는 모터명령(정규화 0~1) -> 로터 각속도 -> 추력 -> 토크로 계산되는
     기지(旣知) 값이라 학습 안 함 (motor_torque() 참고). PINN은 tau_disturbance만 추정.
     omega_dot_measured = 유한차분(실측 각속도, dt)
     physics_loss_rot = MSE(omega_dot_pred, omega_dot_measured)

주의(중요) 1 - 좌표계: Gazebo world wind는 ENU(x=East, y=North) 기준으로 설정했지만,
드론 상태(vn, ve)는 PX4 local NED(x=North, y=East) 기준. 따라서 physics_residual()에서
반드시 축 변환(wind_north = wind_pred_enu[:,1], wind_east = wind_pred_enu[:,0])을
거친 뒤 비교해야 함.

주의(중요) 2 - AD가 아니라 유한차분을 쓰는 이유: 세미나 자료의 PINN 예제(감쇠 단진자)는
순문제(물리상수·초기조건을 다 알고 신경망이 θ(t) 자체를 출력)라 자동미분(AD)으로
θ',θ''을 직접 계산함. 이 프로젝트는 역문제(실측 센서데이터로 안 보이는 외란을 추정 -
신경망 출력은 상태가 아니라 외란값)라, omega_dot/가속도는 신경망 출력의 미분이 아니라
실측 데이터 자체의 미분이 필요함 - AD로 구할 수 있는 값이 아니라 유한차분이 기술적으로
맞는 접근.

yaw_decompose()는 roll/pitch(기체 좌표계, 오일러각)에만 적용됨 - 각속도(wx,wy,wz)는
이미 body frame 벡터라 yaw에 종속적이지 않아 그대로 씀 (12-10절 yaw 일반화 문제가
각속도엔 재발하지 않음).
"""

import numpy as np
import torch
import torch.nn as nn

# ============================================================
# 하이퍼파라미터 - 수정은 여기서
# ============================================================
WINDOW = 20           # 최근 몇 스텝(0.05s 간격이면 1.0초)을 입력으로 쓸지.
                       # gust 데이터(주기 4~10초) 추가 후 10(0.5초)으로는 추세를 읽기엔
                       # 정보가 부족해 보여서 20(1.0초)으로 늘림.
HIDDEN = 64            # 128로 키웠다가 극심한 과적합(train_loss~0, val 전혀 개선 안 됨)
                       # 확인하고 64로 되돌림 - 지금 데이터양엔 이게 맞음. 회전 항 추가로
                       # 입출력이 늘었지만, 일단 이 크기로 되는지부터 확인(검증 전 선확대 안 함)
LAMBDA_PHYSICS = 0.05      # data_loss 대비 병진 physics_loss 가중치
# 회전 쪽은 병진과 성격이 다름: 바람(병진)은 "숨은 미지수"라 정답 라벨(gz topic으로
# 직접 설정한 값)과 물리식(항력식)이 서로 독립된 정보를 줌. 반면 외란토크는 실측
# 각속도(ω)와 계산된 tau_motor만 있으면 tau_disturbance = J*omega_dot - tau_motor로
# 그 자리에서 바로 구해지는 값이라 따로 정답 라벨이 없음 - 물리식 자체가 라벨 역할을
# 겸함. 그래서 회전 쪽엔 "data_loss 대비 physics_loss 비율"이 아니라 "전체 손실에서
# 회전 항이 차지하는 비중"을 뜻함.
LAMBDA_ROT = 0.05  # 일단 LAMBDA_PHYSICS와 동일값 - 손실 스케일이 실제로 다를 수 있어
                   # 학습 로그 보고 재조정 필요
EPOCHS = 400
LR = 1e-3
BATCH_SIZE = 256       # 풀배치 대신 미니배치 - 이질적인 두 데이터(고정/gust)를 섞어
                       # 학습할 때 그래디언트가 "전체 평균"으로 뭉개지는 것 완화
WEIGHT_DECAY = 1e-3    # 과적합 억제용 L2 정규화
VAL_FRACTION = 0.2     # (k-fold 안 쓸 때) condition 단위로 분리

# pos_err(위치오차)는 feature에서 제외함: 보정이 켜지면 pos_err의 일부가 "보정 자신이
# setpoint를 움직여서 생긴 추종 지연"이 되어버려, 모델 출력이 자기 입력에 다시 영향을
# 주는 폐루프가 생기고 학습 범위 밖(pos_err가 원래 <0.5m인데 보정 중엔 수 m까지 커짐)
# 으로 나가면서 폭주함 (pinn_wind_correction_test.py 1차 실험에서 실제로 발산 확인됨).
# vn/ve/roll/pitch는 보정에 의해 setpoint가 바로 바뀌어도 pos_err만큼 직접적/즉각적으로
# 오염되지 않아 상대적으로 안전함.
# roll_deg/pitch_deg 원본 대신 yaw와의 곱항 4개를 씀 (yaw_decompose 참고) -
# 정확한 결합 계수는 신경망이 배움. wx/wy/wz(각속도)는 변환 없이 그대로 추가(위 docstring
# 참고 - body frame 벡터라 yaw 종속성 문제가 없음).
FEATURES = ["vn_m_s", "ve_m_s", "roll_cos_yaw", "roll_sin_yaw", "pitch_cos_yaw", "pitch_sin_yaw",
            "wx_rad_s", "wy_rad_s", "wz_rad_s"]

# ============================================================
# 회전(로터/모터) 물리 상수 - PX4-Autopilot의 gz_x500 실측값
# ============================================================
# 관성모멘트(대각, kg*m^2): Tools/simulation/gz/models/x500_base/model.sdf의 base_link
J_XX, J_YY, J_ZZ = 0.02167, 0.02167, 0.04

# 모터(로터) 추력/토크 모델: Tools/simulation/gz/models/x500/model.sdf의
# gz-sim-multicopter-motor-model 플러그인 실측값
#   추력(N)          = MOTOR_CONST * 각속도^2
#   반작용토크(N*m)   = MOMENT_CONST * 추력  (부호는 회전방향에 따름, ROTOR_YAW_SIGN 참고)
# actuator_output_status().actuator[]가 주는 값 = 로터 각속도(rad/s) 그 자체 (실측
# 검증됨, 2026-08-11: 무풍 호버 조건 로그값을 그대로 이 식에 넣으면 총추력이 mass*g와
# 0.05% 오차로 일치 - EC_MIN/EC_MAX로 다시 정규화할 필요 없음, SIM_GZ_EC 매핑이
# 이미 적용된 값이 나옴).
MOTOR_CONST = 8.54858e-06
MOMENT_CONST = 0.016

# 로터 위치(m, FRD 기준 x=forward,y=right) - PX4-Autopilot airframes/4001_gz_x500의
# CA_ROTORx_PX/PY. 순서: rotor0(전방우측,ccw) rotor1(후방좌측,ccw)
#          rotor2(전방좌측,cw) rotor3(후방우측,cw) - 대각선 쌍이 같은 회전방향(표준 X형)
ROTOR_PX = (0.13, -0.13, 0.13, -0.13)
ROTOR_PY = (0.22, -0.20, -0.22, 0.20)
# ccw 로터는 자신은 ccw로 돌면서 몸체엔 반대(위에서 봤을 때 cw) 반작용 토크를 줌.
# 이 프로젝트가 쓰는 FRD 규약(양의 yaw = 위에서 봤을 때 시계방향)에서 ccw 로터의
# 반작용은 +yaw 기여, cw 로터는 -yaw 기여.
ROTOR_YAW_SIGN = (1.0, 1.0, -1.0, -1.0)

# 검증 완료(2026-08-11, 실제 SITL 소규모 수집): actuator_output_status()는 정규화
# 0~1이 아니라 로터 각속도(rad/s)를 그대로 줌 - 무풍 근처 호버 로그값(약 750~800)을
# 그대로 위 식에 넣으면 총추력이 mass*g와 0.05% 오차로 일치함(motor_torque() 주석 참고).
# 부호(roll/pitch/yaw 방향)는 아직 실측 검증 안 됨 - 학습 후 tau_disturbance가 무풍
# 조건에서 대체로 작게 나오는지로 간접 확인할 것(직접적인 sign 검증은 아니지만, 크게
# 틀렸으면 학습이 아예 수렴 안 하거나 tau_disturbance가 비정상적으로 커질 것으로 예상).


def yaw_decompose(roll_deg, pitch_deg, yaw_deg):
    """roll/pitch를 yaw의 sin/cos와 각각 곱해 4개 항으로 분해.
    대칭을 가정하지 않는 가장 일반적인 1차 후보 기저임 - roll/pitch 축의 게인이
    서로 달라도 신경망의 선형 입력층이 각 항에 다른 가중치를 줄 수 있어 흡수 가능.
    스칼라/numpy 배열 지원."""
    yaw_rad = np.radians(yaw_deg)
    cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
    return roll_deg * cos_y, roll_deg * sin_y, pitch_deg * cos_y, pitch_deg * sin_y


class WindPINN(nn.Module):
    """출력 5개: [wind_vx_enu, wind_vy_enu, tau_dist_x, tau_dist_y, tau_dist_z]
    (기존 병진 출력 2개 + 신규 회전 외란토크 출력 3개, N*m, body frame)."""

    def __init__(self, window, n_features, hidden=HIDDEN):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(window * n_features, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, 5),
        )
        # 항력계수 묶음항 k = 0.5*rho*Cd*A/m (미지수, 데이터로부터 학습)
        self.log_k = nn.Parameter(torch.tensor(-2.0))
        # 회전 쪽엔 이런 학습 가능 상수가 없음 - tau_motor는 모터모델로 이미 정확히
        # 계산되는 기지값이라(motor_torque 참고) PINN은 tau_disturbance(net 출력)만
        # 추정하면 됨.

    def forward(self, x):
        return self.net(x)

    @property
    def k(self):
        return torch.exp(self.log_k)


def physics_residual(model, wind_pred_enu, v_drone_ned):
    """(병진, 변경 없음) wind_pred_enu: (B,2) = [vx_enu(East), vy_enu(North)] 모델 출력의
    앞 2개 열. ENU -> NED 변환 후 항력 방정식(a = k*|v_rel|*v_rel)으로 항력 가속도 예측."""
    wind_north = wind_pred_enu[:, 1]
    wind_east = wind_pred_enu[:, 0]
    v_rel_n = wind_north - v_drone_ned[:, 0]
    v_rel_e = wind_east - v_drone_ned[:, 1]
    speed_rel = torch.sqrt(v_rel_n ** 2 + v_rel_e ** 2 + 1e-6)
    a_pred_n = model.k * speed_rel * v_rel_n
    a_pred_e = model.k * speed_rel * v_rel_e
    return torch.stack([a_pred_n, a_pred_e], dim=1)


def motor_torque(actuator_omega):
    """actuator_omega: (B,4) 로터별 각속도(rad/s, actuator_output_status()가 주는 값을
    그대로 씀 - 정규화 재매핑 불필요, 위 상수 섹션 주석 참고). 순서 rotor0~3.
    Gazebo 모터모델 상수로 몸체좌표 토크(B,3) = [tau_x(roll), tau_y(pitch), tau_z(yaw)]
    계산 (N*m). PINN이 배우는 값이 아니라 상수로 직접 계산하는 기지 물리량."""
    thrust = MOTOR_CONST * actuator_omega ** 2               # (B,4) N

    px = thrust.new_tensor(ROTOR_PX)
    py = thrust.new_tensor(ROTOR_PY)
    yaw_sign = thrust.new_tensor(ROTOR_YAW_SIGN)

    tau_x = -(thrust * py).sum(dim=1)                       # roll:  -sum(PY_i * F_i)
    tau_y = (thrust * px).sum(dim=1)                        # pitch:  sum(PX_i * F_i)
    tau_z = MOMENT_CONST * (thrust * yaw_sign).sum(dim=1)    # yaw: 반작용토크 합
    return torch.stack([tau_x, tau_y, tau_z], dim=1)


def physics_residual_rotation(tau_dist_pred, actuator_omega):
    """tau_dist_pred: (B,3) PINN이 추정한 외란토크(N*m, body frame, 모델 출력의 뒤 3개 열).
    actuator_omega: (B,4) 로터별 각속도(rad/s, motor_torque() 참고).
    J*omega_dot = tau_motor(계산) + tau_disturbance(예측) 이므로
    omega_dot_pred = (tau_motor + tau_disturbance) / J.
    반환값(B,3)은 실측 각속도의 유한차분(train_wind_estimator.py)과 비교하는 용도."""
    tau_motor = motor_torque(actuator_omega)
    tau_total = tau_motor + tau_dist_pred
    j_diag = tau_total.new_tensor([J_XX, J_YY, J_ZZ])
    return tau_total / j_diag
