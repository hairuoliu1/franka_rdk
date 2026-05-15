from lerobot.processor import PolicyProcessorPipeline, PolicyAction
from lerobot.processor.converters import policy_action_to_transition, transition_to_policy_action
from typing import Any
import torch
import numpy as np
from PIL import Image as PILImage
from einops import rearrange
from .configuration_openpi_client import OpenPIClientConfig

# --- 图像处理工具函数 ---
def convert_to_uint8(img: np.ndarray) -> np.ndarray:
    if np.issubdtype(img.dtype, np.floating):
        img = (255 * img).astype(np.uint8)
    return img

def _resize_with_pad_pil(image: PILImage.Image, height: int, width: int, method: int) -> PILImage.Image:
    cur_width, cur_height = image.size
    if cur_width == width and cur_height == height:
        return image
    ratio = max(cur_width / width, cur_height / height)
    resized_height = int(cur_height / ratio)
    resized_width = int(cur_width / ratio)
    resized_image = image.resize((resized_width, resized_height), resample=method)
    zero_image = PILImage.new(resized_image.mode, (width, height), 0)
    pad_height = max(0, int((height - resized_height) / 2))
    pad_width = max(0, int((width - resized_width) / 2))
    zero_image.paste(resized_image, (pad_width, pad_height))
    return zero_image

def resize_with_pad(images: np.ndarray, height: int, width: int, method=PILImage.Resampling.BILINEAR) -> np.ndarray:
    if images.shape[-3:-1] == (height, width):
        return images

    original_shape = images.shape
    images = images.reshape(-1, *original_shape[-3:])
    resized = np.stack([np.array(_resize_with_pad_pil(PILImage.fromarray(im), height, width, method=method)) for im in images])
    return resized.reshape(*original_shape[:-3], *resized.shape[-3:])


class OpenPIDataProcessor:
    def __init__(self, config: OpenPIClientConfig):
        self.config = config

    def forward_process(self, batch: dict[str, torch.Tensor]) -> dict:
        """
        按照 inference_process 的处理流程转换数据。
        """
        payload = {"images": {}}
        
        # 1. 关节状态提取
        # LeRobot 可能会将其组合为 "observation.state"，也有可能散落为 "observation.joint_positions_0" 等
        if "observation.state" in batch:
            payload["state"] = batch["observation.state"][0].cpu().numpy()
        else:
            # 万一没有做自动拼接，我们提供一个 fallback。
            # 这里先尝试抓取关节数据 (0-7)，如果后续还有配置的力矩等，也会一并抓取，但最好依赖 LeRobot 已经拼接好的 observation.state
            joint_values = []
            for i in range(8):
                key = f"observation.joint_positions_{i}"
                if key in batch:
                    joint_values.append(batch[key][0].item())
                else:
                    joint_values.append(0.0)
            payload["state"] = np.array(joint_values, dtype=np.float32)

        # 2. 图像数据转换逻辑 
        image_keys = [k for k in batch.keys() if "image" in k]
        for key in image_keys:
            img_tensor = batch[key][0] # (C, H, W)
            img_np = img_tensor.cpu().numpy()
            
            # 转为 HWC 格式给 resize_with_pad 使用
            img_hwc = rearrange(img_np, 'c h w -> h w c')
            
            # LeRobot 传递进来的是 [0.0, 1.0] 的 float32 张量
            # 使用 PIL 处理之前必须先转成 uint8，否则 PIL.Image.fromarray 会报错
            img_hwc_uint8 = convert_to_uint8(img_hwc)
            
            # 注意: LeRobot 的 get_observation 默认会把图像按 RGB 给。
            # 如果你在录制时由于 ROS 节点吐出 BGR 并且没处理，想在这里强制转换可解开下方注释:
            # img_hwc_uint8 = img_hwc_uint8[:, :, [2, 1, 0]]
            
            # resize并pad
            img_resized = resize_with_pad(img_hwc_uint8, 224, 224)
            
            # 转为 CHW
            img_chw = rearrange(img_resized, 'h w c -> c h w')
            
            short_key = key.split('.')[-1]
            payload["images"][short_key] = img_chw

        # 3. 填入 prompt 供服务端使用
        payload["prompt"] = getattr(self.config, "default_prompt", "")
        
        return payload

    def backward_process(self, action_chunk: np.ndarray) -> np.ndarray:
        # 如以后需要动作插值等逻辑可以加在这里
        return action_chunk

def make_openpi_client_pre_post_processors(
    config: OpenPIClientConfig,
    dataset_stats: dict[str, dict[str, torch.Tensor]] | None = None,
) -> tuple[
    PolicyProcessorPipeline[dict[str, Any], dict[str, Any]],
    PolicyProcessorPipeline[PolicyAction, PolicyAction],
]:
    # 返回LeRobot要求的壳，核心运算挂载在本地的 OpenPIDataProcessor 里
    return (
        PolicyProcessorPipeline([]),
        PolicyProcessorPipeline(
            [],
            to_transition=policy_action_to_transition,
            to_output=transition_to_policy_action,
        ),
    )
