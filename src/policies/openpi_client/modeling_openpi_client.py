import torch
import numpy as np
import requests
import msgpack
from typing import Dict
from torch import Tensor
from torch.nn import Linear
from lerobot.policies.pretrained import PreTrainedPolicy
from .configuration_openpi_client import OpenPIClientConfig
from .processor_openpi_client import OpenPIDataProcessor

# --- MsgPack Helper Functions ---
def pack_array(obj):
    if (isinstance(obj, (np.ndarray, np.generic))) and obj.dtype.kind in ("V", "O", "c"):
        raise ValueError(f"Unsupported dtype: {obj.dtype}")
    if isinstance(obj, np.ndarray):
        return {b"__ndarray__": True, b"data": obj.tobytes(), b"dtype": obj.dtype.str, b"shape": obj.shape}
    if isinstance(obj, np.generic):
        return {b"__npgeneric__": True, b"data": obj.item(), b"dtype": obj.dtype.str}
    return obj

def unpack_array(obj):
    if b"__ndarray__" in obj:
        return np.ndarray(buffer=obj[b"data"], dtype=np.dtype(obj[b"dtype"]), shape=obj[b"shape"])
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj

def packb(data):
    return msgpack.packb(data, default=pack_array, use_bin_type=True)


def unpackb(data):
    return msgpack.unpackb(data, object_hook=unpack_array, raw=False)


class OpenPIClientPolicy(PreTrainedPolicy):
    config_class = OpenPIClientConfig
    name = "openpi_client"
    
    def __init__(self, config: OpenPIClientConfig):
        super().__init__(config)
        self.config = config
        

        self.base_url = config.host.rstrip('/')
        
        self.infer_url = f"{self.base_url}/infer"
        self.session = requests.Session()
        self.session.headers.update({"Content-Type": "application/msgpack"})
        
        self.template_model = Linear(1, 1)
        self.processor = OpenPIDataProcessor(self.config)
        self.reset()

    def reset(self):
        self._cur_step: int = 0
        self._last_results = None

    @torch.no_grad()
    def select_action(self, batch: dict[str, Tensor]) -> Tensor:
        if self._last_results is None:
            self._last_results = self.predict_action_chunk(batch).cpu().numpy()
            if len(self._last_results) == 0:
                raise RuntimeError("OpenPI server returned an empty action chunk.")
            self._cur_step = 0

        # 取出当前步的Action
        action = self._last_results[self._cur_step]
        self._cur_step += 1
        
        # 只执行 n_action_steps 步，之后用最新 observation 重新请求预测。
        max_cached_steps = min(self.config.n_action_steps, len(self._last_results))
        if self._cur_step >= max_cached_steps:
            self._last_results = None

        return torch.tensor(action, dtype=torch.float32)

    @torch.no_grad()
    def predict_action_chunk(self, batch: dict[str, Tensor]) -> Tensor:
        # 1. 在本地进行图像转换、Resize_With_pad、uint8转化等预处理
        payload = self.processor.forward_process(batch)

        # 2. 推送给远程 OpenPI 服务端
        action_chunk = self._post(payload)

        # 3. 后处理（如果需要插值等可以写在里面）
        action_chunk = self.processor.backward_process(action_chunk)
        if len(action_chunk) == 0:
            raise RuntimeError("OpenPI server returned an empty action chunk.")

        return torch.tensor(action_chunk, dtype=torch.float32)

    def _post(self, payload: Dict) -> np.ndarray:
        try:
            packed_observation = packb(payload)
            resp = self.session.post(self.infer_url, data=packed_observation, timeout=30)
            resp.raise_for_status()
            data = unpackb(resp.content)
            
            # 从服务器返回的数据中拿到actions
            action = np.array(data) if not isinstance(data, dict) else np.array(data.get("actions", data))
            return action
        except Exception as e:
            raise RuntimeError(f"Policy server request failed: {e}") from e

    def forward(self, batch): pass
    def get_optim_params(self): pass
