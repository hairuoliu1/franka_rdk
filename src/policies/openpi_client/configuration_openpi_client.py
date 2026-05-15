from dataclasses import dataclass
from lerobot.configs.policies import PreTrainedConfig
from lerobot.optim.optimizers import AdamConfig
from lerobot.optim.schedulers import DiffuserSchedulerConfig

@PreTrainedConfig.register_subclass("openpi_client")
@dataclass
class OpenPIClientConfig(PreTrainedConfig):
    # 策略的视野和预测动作步数
    n_obs_steps: int = 1
    horizon: int = 50
    n_action_steps: int = 25

    # 服务端连接配置
    host: str = "http://127.0.0.1:8000"
    
    # 指令Prompt配置
    default_prompt: str = ""
    
    def __post_init__(self):
        super().__post_init__()

    @property
    def observation_delta_indices(self) -> list:
        return list(range(1 - self.n_obs_steps, 1))

    @property
    def action_delta_indices(self) -> list:
        return list(range(1 - self.n_obs_steps, 1 - self.n_obs_steps + self.horizon))

    @property
    def reward_delta_indices(self) -> None:
        return None

    def get_optimizer_preset(self) -> AdamConfig | None:
        return None

    def get_scheduler_preset(self) -> DiffuserSchedulerConfig | None:
        return None

    def validate_features(self) -> None:
        return
