"""
PINN 바람 추정 모델의 "물리/모델 정의" 부분만 모아둔 파일.
방정식이나 하이퍼파라미터를 고치고 싶으면 여기만 보면 됨 - 데이터 로딩/학습 루프
(train_wind_estimator.py)나 실제 비행 제어(pinn_wind_correction_*.py)와 분리해뒀음.

물리 제약(PINN residual)의 형태:
  v_rel = wind_pred(NED) - v_drone(NED)
  a_drag_pred = k * |v_rel| * v_rel      (k = 0.5*rho*Cd*A/m, 학습 가능한 스칼라)
  a_measured  = 유한차분(v_drone, dt)
  physics_loss = MSE(a_drag_pred, a_measured)

주의(중요): Gazebo world wind는 ENU(x=East, y=North) 기준으로 설정했지만, 드론 상태
(vn, ve)는 PX4 local NED(x=North, y=East) 기준. 따라서 physics_residual()에서 반드시
축 변환(wind_north = wind_pred_enu[:,1], wind_east = wind_pred_enu[:,0])을 거친 뒤
비교해야 함.

yaw_decompose()는 별개의 "물리적으로 그럴듯한 후보 기저" - roll/pitch(기체 좌표계)를
yaw의 sin/cos와 곱해 4개 항으로 분해해서, 대칭을 가정하지 않고도 신경망이 정확한
결합 방식을 데이터로부터 배우게 함 (train_wind_estimator.py 모듈 docstring의 "주의 2"
참고).
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
                       # 확인하고 64로 되돌림 - 지금 데이터양엔 이게 맞음
LAMBDA_PHYSICS = 0.05  # data_loss 대비 physics_loss 가중치
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
# 정확한 결합 계수는 신경망이 배움.
FEATURES = ["vn_m_s", "ve_m_s", "roll_cos_yaw", "roll_sin_yaw", "pitch_cos_yaw", "pitch_sin_yaw"]


def yaw_decompose(roll_deg, pitch_deg, yaw_deg):
    """roll/pitch를 yaw의 sin/cos와 각각 곱해 4개 항으로 분해.
    대칭을 가정하지 않는 가장 일반적인 1차 후보 기저임 - roll/pitch 축의 게인이
    서로 달라도 신경망의 선형 입력층이 각 항에 다른 가중치를 줄 수 있어 흡수 가능.
    스칼라/numpy 배열 지원."""
    yaw_rad = np.radians(yaw_deg)
    cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
    return roll_deg * cos_y, roll_deg * sin_y, pitch_deg * cos_y, pitch_deg * sin_y


class WindPINN(nn.Module):
    def __init__(self, window, n_features, hidden=HIDDEN):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(window * n_features, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, 2),
        )
        # 항력계수 묶음항 k = 0.5*rho*Cd*A/m (미지수, 데이터로부터 학습)
        self.log_k = nn.Parameter(torch.tensor(-2.0))

    def forward(self, x):
        return self.net(x)

    @property
    def k(self):
        return torch.exp(self.log_k)


def physics_residual(model, wind_pred_enu, v_drone_ned):
    """wind_pred_enu: (B,2) = [vx_enu(East), vy_enu(North)] 모델 출력(라벨과 동일 표기).
    ENU -> NED 변환 후 항력 방정식(a = k*|v_rel|*v_rel)으로 항력 가속도 예측.
    항력 방정식 자체를 다른 걸로 바꾸고 싶으면 이 함수만 고치면 됨."""
    wind_north = wind_pred_enu[:, 1]
    wind_east = wind_pred_enu[:, 0]
    v_rel_n = wind_north - v_drone_ned[:, 0]
    v_rel_e = wind_east - v_drone_ned[:, 1]
    speed_rel = torch.sqrt(v_rel_n ** 2 + v_rel_e ** 2 + 1e-6)
    a_pred_n = model.k * speed_rel * v_rel_n
    a_pred_e = model.k * speed_rel * v_rel_e
    return torch.stack([a_pred_n, a_pred_e], dim=1)
