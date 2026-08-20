"""correction_experiments 안의 4개 스크립트(`pinn_correction_param_tuning.py`,
`pinn_wind_correction_sweep.py`, `pinn_wind_correction_gust_sweep.py`,
`pinn_rotation_correction_test.py`)에 토씨 하나 안 틀리고 복붙돼있던 유틸(바람 설정,
PINN 추정 버퍼, 스무딩, deadband)을 하나로 모음(EXPERIMENTS.md 12-21절 2번,
12-31/12-33절 - 복붙 상태 때문에 `set_wind()` 재발행 버그를 두 파일에서 따로
발견/수정해야 했던 게 이 정리의 직접적인 계기).

순수 리팩터링 - 로직/동작은 원본과 동일하게 유지함.
"""
import asyncio
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "offline_training"))
from wind_pinn_model import WindPINN  # noqa: E402


async def set_wind(world_name, vx, vy, vz=0.0):
    """gz topic pub은 1회성 CLI라, gz-transport 구독자 탐색(discovery, 보통
    수백ms 걸림)이 끝나기 전에 프로세스가 끝나면 CLI는 성공(exit 0)을 보고해도
    실제로 아무도 못 받을 수 있음 - 25회 반복측정 중 1회 실제로 발생 확인됨
    (EXPERIMENTS.md 12-23절). 같은 값(멱등)을 짧은 간격으로 3번 재전송해서
    완화 - 한 번이라도 discovery 이후에 도착하면 됨."""
    last_returncode = None
    for attempt in range(3):
        proc = await asyncio.create_subprocess_exec(
            "gz", "topic", "-t", f"/world/{world_name}/wind",
            "-m", "gz.msgs.Wind",
            "-p", f"linear_velocity: {{x: {vx}, y: {vy}, z: {vz}}}, enable_wind: true",
        )
        await proc.wait()
        last_returncode = proc.returncode
        if attempt < 2:
            await asyncio.sleep(0.15)
    if last_returncode != 0:
        # 조용히 넘기면 바람이 실제로 안 걸린 채 트라이얼이 진행될 수 있음 - 실제로
        # 이걸로 엉터리 결과가 나온 적 있어(EXPERIMENTS.md 12-17/12-21절) 예외로 드러냄.
        raise RuntimeError(f"gz topic pub 실패 (returncode={last_returncode})")


class PINNCorrector:
    """WindPINN 체크포인트 로드 + 슬라이딩 윈도우 버퍼 + 추론.
    출력 인덱싱(풍속 north/east인지 tau_x/y/z인지)은 호출부 책임으로 남김 - 4개
    스크립트가 같은 체크포인트의 서로 다른 출력 채널(풍속 2채널 vs 회전토크 3채널)을
    쓰기 때문에, 여기서는 원시 예측값(`predict_raw()`)만 제공함."""

    def __init__(self, model_path):
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        self.model = WindPINN(ckpt["window"], len(ckpt["features"]))
        self.model.load_state_dict(ckpt["model_state"])
        self.model.eval()
        self.X_mean = torch.tensor(ckpt["X_mean"], dtype=torch.float32)
        self.X_std = torch.tensor(ckpt["X_std"], dtype=torch.float32)
        self.window = ckpt["window"]
        self.features = ckpt["features"]
        self.buffer = []

    def push_state(self, feat_vec):
        self.buffer.append(feat_vec)
        if len(self.buffer) > self.window:
            self.buffer.pop(0)

    def ready(self):
        return len(self.buffer) == self.window

    @torch.no_grad()
    def predict_raw(self):
        x = torch.tensor(self.buffer, dtype=torch.float32).flatten()
        xn = (x - self.X_mean) / self.X_std
        return self.model(xn.unsqueeze(0)).squeeze(0)


class EmaSmoother:
    def __init__(self, time_constant_s, dt_s):
        self.alpha = dt_s / (time_constant_s + dt_s)
        self.value = None

    def update(self, x):
        self.value = x if self.value is None else self.alpha * x + (1 - self.alpha) * self.value
        return self.value


def apply_deadband(wind_n, wind_e, deadband):
    """speed<=deadband면 (0,0), 그 이상이면 방향 유지한 채 크기만
    (speed-deadband)만큼으로 줄임 - 문턱값에서 뚝 끊기지 않고 연속적으로 이어짐."""
    speed = (wind_n ** 2 + wind_e ** 2) ** 0.5
    if speed <= deadband or speed == 0.0:
        return 0.0, 0.0
    scale = (speed - deadband) / speed
    return wind_n * scale, wind_e * scale
