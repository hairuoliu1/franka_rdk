# Franka FR3 + GELLO + Robotiq 遥操作平台

基于 ROS 2 Humble 的 Franka FR3 遥操作系统，使用 GELLO 作为主端控制器，Robotiq 2F-85 作为夹爪。

## 硬件组成

- Franka FR3 机械臂
- GELLO 遥操作手柄 (Dynamixel XL330-M288, U2D2/FTDI 转换器)
- Robotiq 2F-85 夹爪
- Robotiq 六维力传感器
- Intel RealSense D405 + D415 相机

---

## 项目结构

```
workspace/
├── ros2/src/                            # ROS 2 包
│   ├── franka_fr3_arm_controllers/      # FR3 关节阻抗控制器
│   ├── franka_gello_state_publisher/    # GELLO 状态发布节点
│   ├── franka_gripper_manager/          # 夹爪管理器 (Robotiq)
│   └── rq_fts_ros2_driver/              # Robotiq FT 传感器驱动
├── src/
│   ├── config/                          # 各类配置文件
│   ├── robots/                          # LeRobot 机器人接口
│   ├── teleoperators/                   # LeRobot 遥操作接口
│   └── policies/                        # 策略模型 (OpenPI)
├── tmp/                                 # 测试脚本
│   ├── test_action_listener.py          # 监听 ROS action
│   ├── fake_robot_action_publisher.py   # 模拟发送 action
│   ├── test_obs_listener.py             # 监听 ROS observation
│   └── fake_robot_state_publisher.py    # 模拟发送 observation
└── lerobot/                             # LeRobot 框架 (git submodule)
```

> **lerobot 为 git submodule**，指向上游 [huggingface/lerobot](https://github.com/huggingface/lerobot)，本地修改通过 `patches/lerobot.patch` 管理。详见[子模块管理](#子模块管理)。

---

## Git 仓库与子模块管理

### 克隆

```bash
git clone --recursive <repo-url>
# 如果已 clone 但忘记 --recursive：
git submodule update --init
```

### 应用本地补丁

补丁基于上游 commit `818892a38bbf` 生成，必须先让 submodule 回到该 commit：

```bash
# submodule update 会自动 checkout 到父仓库记录的 commit (818892a38bbf)
git submodule update --init lerobot

# 确认在正确 commit
cd lerobot && git log -1 --oneline
# 818892a3 feat(dagger): Add HIL/Dagger/HG-Dagger/RaC style data collection (#2833)

# 应用本地补丁
git apply ../patches/lerobot.patch
```

> 如果你 `cd lerobot && git pull` 到了更新的版本，patch 可能 conflict。此时需要先 `git checkout 818892a38bbf` 回到基准 commit 再 apply。

### 修改 lerobot 后更新补丁

```bash
cd lerobot
# ... 正常开发、git commit 你的修改 ...
git diff 818892a3 > ../patches/lerobot.patch   # 导出相对于上游的补丁
cd /workspace && git add patches/lerobot.patch && git commit -m "update lerobot patch"
```

> 上游基准 commit：`818892a38bbfaa4c3ce7597d0db4504d730e51c7`（`main` 分支，`feat(dagger): Add HIL/Dagger/HG-Dagger/RaC style data collection (#2833)`）。如需跟踪更新版本，在 lerobot 内 `git pull` 后重新生成 patch。

### 补丁内容概要

| 文件 | 修改原因 |
|---|---|
| `pyproject.toml` | `requires-python` 从 3.12 降为 3.10 |
| `motors_bus.py` | 类型注解兼容 Python 3.10 (`Union` 替代 `\|`) |
| `policies/` | 路径适配、模型加载兼容 |
| `cameras/realsense/` | ROS 相机接口适配 |
| `datasets/streaming_dataset.py` | 数据集路径调整 |

---

## 配置文件

所有配置文件集中存放在 `/workspace/src/config/`：

| 文件 | 用途 |
|---|---|
| `fr3_config.yaml` | FR3 控制器配置 (IP、namespace 等) |
| `gello_publisher.yaml` | GELLO 配置 (端口、偏移量、关节方向) |
| `robotiq_gripper_config.yaml` | Robotiq 夹爪配置 (端口、namespace) |

### fr3_config.yaml 示例

```yaml
robot1:
  arm_id: "fr3"
  arm_prefix: ""
  fake_sensor_commands: "false"
  joint_sources: ["joint_states", "franka_gripper/joint_states"]
  joint_state_rate: 30
  load_gripper: "true"
  namespace: ""
  robot_ip: "172.16.0.3"
  urdf_file: "fr3/fr3.urdf.xacro"
  use_fake_hardware: "false"
  use_rviz: "false"
```

### gello_publisher.yaml 示例

```yaml
SINGLE:
  namespace: ""
  com_port: "usb-FTDI_USB__-__Serial_Converter_FTAWANP9-if00-port0"
  num_arm_joints: 7
  joint_signs: [1, -1, 1, -1, 1, -1, 1]
  gripper: true
  assembly_offsets: [3.142, 0.0, 3.142, 4.712, 3.142, 1.571, -0.8]  # rad
  gripper_range_rad: [3.914, 5.134]
  dynamixel_torque_enable: [0, 0, 0, 0, 0, 0, 0, 0]
  dynamixel_goal_position: [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.0, 3.509]
  dynamixel_kp_p: [30, 60, 0, 30, 0, 0, 0, 50]
  dynamixel_kp_i: [0, 0, 0, 0, 0, 0, 0, 0]
  dynamixel_kp_d: [250, 100, 80, 60, 30, 10, 5, 0]
```

---

## Python 环境说明

LeRobot 最新版要求 Python 3.12，但 ROS 2 默认使用 Python 3.10。为避免冲突，将 UV 环境设为 3.10：

1. 修改 `lerobot/pyproject.toml` 中 `requires-python = ">=3.10"`
2. 修改 `lerobot/src/lerobot/motors/motors_bus.py` 中的类型注解为兼容写法：

```python
# 替换前 (仅 3.12+ 支持):
# type NameOrID = str | int
# type Value = int | float

# 替换后 (兼容 3.10):
from typing import Union
NameOrID = Union[str, int]
Value = Union[int, float]
```

常用命令：

```bash
# 查看已安装的包
/workspace/.venv/bin/python -m pip list

# 激活虚拟环境
source /workspace/.venv/bin/activate

# 清华镜像（已写入 ~/.bashrc）
export UV_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
```

---

## 遥操作

### 步骤 1：查找串口 ID

```bash
ls /dev/serial/by-id/
```

典型输出：
- **GELLO (U2D2 转换器)**：`usb-FTDI_USB__-__Serial_Converter_FTAWANP9-if00-port0`
- **Robotiq 夹爪**：`usb-FTDI_USB_TO_RS-485_DAAQMJL8-if00-port0`

根据找到的端口更新 `gello_publisher.yaml` 和 `robotiq_gripper_config.yaml` 中的 `com_port` 字段。

### 步骤 2：校准 GELLO 偏移量

将 GELLO 摆到与 Franka 起始姿态相同的位置，运行偏移量计算脚本：

```bash
cd /workspace/ros2/src/franka_gello_state_publisher/scripts/

python3 get_offsets.py \
    --start-joints 0 0 0 -1.57 0 1.57 0 \
    --joint-signs 1 -1 1 -1 1 1 1 \
    --port /dev/serial/by-id/<你的GELLO端口ID>
```

参数说明：
- `--start-joints`：标定姿势下各关节目标角度（弧度），joint1 ~ joint7
- `--joint-signs`：电机方向符号，1 为正，-1 为反
- `--port`：步骤 1 中找到的 GELLO 端口

将输出的 `assembly_offsets`、`gripper_range_rad` 等值更新到 `gello_publisher.yaml`。

### 步骤 3：构建 ROS 2 工作空间

```bash
cd /workspace/ros2
colcon build
source install/setup.bash
```

### 步骤 4：启动 RealSense 相机

```bash
# 安装 (首次)
sudo apt install -y ros-humble-realsense2-camera

# 查看相机序列号
rs-enumerate-devices | grep "Serial Number"
# 或使用 lerobot 工具：
lerobot-find-cameras realsense

# D405 (cam1)
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=cam1 camera_name:=rs1 serial_no:="'352122272067'"

# D415 (cam2)
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=cam2 camera_name:=rs2 serial_no:="'311122062207'"
```

验证：

```bash
ros2 topic list | grep -E "cam1|cam2|color/image"
```

### 步骤 5：启动 GELLO 状态发布器

> **必须先启动 GELLO，再启动 Franka 控制器。** 否则 joint_impedance_controller 收不到有效的关节状态，机械臂会因目标位置跳变触发 reflex 保护 (power_limit_violation / joint_velocity_violation)。

```bash
ros2 launch franka_gello_state_publisher main.launch.py \
    config_file:=/workspace/src/config/gello_publisher.yaml
```

验证 GELLO 正常发布：

```bash
ros2 topic echo /left/gello/joint_states
ros2 topic echo /right/gello/joint_states
# 确认关节值在正常范围内 (不应出现 ±2.9007 等极限值)
```

### 步骤 6：启动 Franka 机械臂

> 确认 GELLO 正常发布后，再启动此步骤。

```bash
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
    robot_config_file:=/workspace/src/config/fr3_config.yaml
```

启动后的话题列表：

```
/dynamic_joint_states
/franka/joint_states
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

查看末端位姿：

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_link8
# 示例输出:
# - Translation: [0.307, 0.000, 0.590]
# - Rotation: in RPY (degree) [-180.000, 0.000, -45.000]
```

移动到预设起始位置：

```bash
ros2 launch franka_bringup example.launch.py \
  controller_name:=move_to_start_example_controller \
  robot_config_file:=./franka.config.yaml
```

### 步骤 7：启动 Robotiq 夹爪管理器

```bash
# 确认串口存在
ls -l /dev/serial/by-id

# 启动夹爪控制节点
ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
    config_file:=/workspace/src/config/robotiq_gripper_config.yaml
```

夹爪测试：

```bash
python -m src.robots.franka_fr3_robotiq_gripper.gripper_test
```

Robotiq 2F-85 关节范围：`0.0` = 完全张开，`0.8` = 完全闭合（弧度制）。实际闭合读数约 `0.7894`。

### 步骤 9：启动力传感器（可选）

```bash
# 编译 (首次)
cd /workspace/ros2/src
git clone https://github.com/panagelak/rq_fts_ros2_driver.git
cd /workspace/ros2
colcon build --packages-select robotiq_ft_sensor_hardware robotiq_ft_sensor_description robotiq_controllers
source install/setup.bash

# 启动
ros2 launch robotiq_ft_sensor_hardware ft_sensor_standalone.launch.py

# 观测
ros2 topic echo /robotiq_force_torque_sensor_broadcaster/wrench
```

---

## 话题拓扑

```
RealSense D405/D415 ──→ /cam1/rs1/color/image_rect_raw
                         /cam2/rs2/color/image_raw

GELLO (Dynamixel) ──→ gello_publisher ──→ /gello/joint_states
                                               │
                                               ▼
                         joint_impedance_controller ──→ Franka FR3 (力矩)

                         gripper_manager ──→ Robotiq 2F-85 (串口)

Robotiq FT Sensor ──→ /robotiq_force_torque_sensor_broadcaster/wrench
```

> 注意：启动全部节点后的话题列表会多出 `/gello/joint_states`、`/franka_gripper/joint_states`、`/joint_impedance_controller/transition_event` 等。

---

## 关节状态读取约定

**ROS 发布的 joint topic 中关节名称顺序不固定，切勿用 list 索引直接读取，必须通过字典查找。**

实际观察到的 `/franka/joint_states` 顺序（注意 joint2 和 joint3 是反的）：

```
name:
- fr3_joint1
- fr3_joint3    ← 注意位置
- fr3_joint2
- fr3_joint4
- fr3_joint5
- fr3_joint6
- fr3_joint7
```

正确读取方式：

```python
arm_positions = []
for i in range(1, 8):
    joint_name = f"fr3_joint{i}"
    if joint_name in self._joint_state_msg.name:
        idx = self._joint_state_msg.name.index(joint_name)
        arm_positions.append(self._joint_state_msg.position[idx])
    else:
        arm_positions.append(0.0)
arm = np.array(arm_positions, dtype=np.float32)
```

此约定适用于所有 ROS 话题的读写（observation、action、joint_states 等）。

---

## 数据录制 (LeRobot Record)

### 架构说明

遥操作通过 ROS 2 完成，LeRobot Record 运行在**旁路监听模式 (passive mode)**：仅从 ROS 话题读取 action 和 observation 用于记录，不向机械臂下发指令。

```python
# lerobot_record.py 中的旁路监听逻辑
if policy is None and getattr(teleop, "is_passive", False):
    # 仅记录 action, 不向机械臂下发, 避免与 ROS 控制冲突
    _sent_action = robot_action_to_send
else:
    _sent_action = robot.send_action(robot_action_to_send)
```

### 放行 X11 图形权限

```bash
xhost +local:docker
# 或
xhost +
```

### 启动录制

录制前确保所有 ROS 节点均已启动（步骤 4-9）。

无相机录制：

```bash
python -m src.lerobot_record \
    --robot.type=franka_fr3_robotiq_gripper \
    --teleop.type=gello_ros_leader \
    --dataset.repo_id=local/my_franka_dataset \
    --dataset.root=/workspace/data/franka_exam1 \
    --dataset.num_episodes=2 \
    --dataset.single_task="Pick up the yellow block" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true \
    --play_sounds=false \
    --dataset.push_to_hub=false \
    --dataset.vcodec=h264 \
    --dataset.fps=15 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=0
```

带相机录制：

```bash
python -m src.lerobot_record \
    --robot.type=franka_fr3_robotiq_gripper \
    --robot.cameras='{top: {type: intelrealsense, serial_number_or_name: "311122062207", width: 640, height: 480, fps: 30}}' \
    --teleop.type=gello_ros_leader \
    --dataset.repo_id=local/my_franka_dataset_top_only \
    --dataset.root=/workspace/data/franka_exam_top_only \
    --dataset.num_episodes=2 \
    --dataset.single_task="Pick up the yellow block" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=1 \
    --dataset.encoder_queue_maxsize=120 \
    --display_data=false \
    --play_sounds=false \
    --dataset.push_to_hub=false \
    --dataset.vcodec=h264 \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=0
```

### 录制控制快捷键

录制过程中通过键盘控制：

| 按键 | 作用 |
|------|------|
| **→ 右方向键** | 提前结束当前循环 (`exit_early = True`) |
| **← 左方向键** | 结束当前循环并标记重录该 episode (`rerecord_episode = True`) |
| **Esc** | 停止整个录制流程 (`stop_recording = True`) |

### 可视化录制数据

```bash
python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id local/my_franka_dataset \
  --episode-index 0 \
  --root /workspace/data/franka_exam1
```

### Observation/Action 测试工具

`tmp/` 目录下的测试脚本用于验证 ROS 通信：

| 脚本 | 用途 |
|------|------|
| `tmp/test_action_listener.py` | 监听 ROS 发布的 action |
| `tmp/fake_robot_action_publisher.py` | 模拟发送 action |
| `tmp/test_obs_listener.py` | 监听 ROS 发布的 observation |
| `tmp/fake_robot_state_publisher.py` | 模拟发送 observation |

```bash
PYTHONPATH=/workspace python /workspace/tmp/test_gello_listener.py
# 或模块模式
python -m tmp.test_gello_listener
```

---

## 模型远程推理 (OpenPI)

```bash
python -m src.lerobot_record \
    --robot.type=franka_fr3_robotiq_gripper \
    --policy.type=openpi_client \
    --policy.host=http://127.0.0.1:8000 \
    --policy.default_prompt="Pick up the yellow block" \
    --dataset.repo_id=local/my_franka_policy_dataset \
    --dataset.root=/workspace/data/franka_policy_exam1 \
    --dataset.num_episodes=2 \
    --dataset.single_task="Pick up the yellow block" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true \
    --play_sounds=false \
    --dataset.push_to_hub=false \
    --dataset.vcodec=h264 \
    --dataset.fps=15 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=0
```

---

## 一键启动脚本

5 个终端分别执行：

```bash
# ===== 终端 1: GELLO 状态发布 =====
ros2 launch franka_gello_state_publisher main.launch.py \
    config_file:=/workspace/src/config/gello_publisher.yaml

# ===== 终端 2: 关节阻抗控制器 =====
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
    robot_config_file:=/workspace/src/config/fr3_config.yaml

# ===== 终端 3: Robotiq 夹爪 =====
ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
    config_file:=/workspace/src/config/robotiq_gripper_config.yaml

# ===== 终端 4: 力传感器 (可选) =====
ros2 launch robotiq_ft_sensor_hardware ft_sensor_standalone.launch.py

# ===== 终端 5: 数据录制 =====
python -m src.lerobot_record \
    --robot.type=franka_fr3_robotiq_gripper \
    --teleop.type=gello_ros_leader \
    --dataset.repo_id=local/my_franka_dataset \
    --dataset.root=/workspace/data/franka_exam1 \
    --dataset.num_episodes=5 \
    --dataset.single_task="Pick up the yellow block" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true \
    --play_sounds=false \
    --dataset.push_to_hub=false \
    --dataset.vcodec=h264 \
    --dataset.fps=15 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=0
```

---

## Docker 配置

### docker-compose.yml 关键修改

相比 GELLO 原始配置，新增串口和 USB 透传：

```yaml
services:
  gello-ros2:
    build: .
    privileged: true
    network_mode: host
    init: true
    stop_signal: SIGINT
    volumes:
      - ../../:/workspace
      - /dev/serial/by-id:/dev/serial/by-id
      - /dev/bus/usb:/dev/bus/usb    # USB 透传
      - /dev:/dev                      # 设备文件透传
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /tmp/.docker.xauth:/tmp/.docker.xauth:rw
    environment:
      DISPLAY: ${DISPLAY}
      QT_X11_NO_MITSH: "1"
      XAUTHORITY: /tmp/.docker.xauth
```

### Dockerfile 关键修改

- 替换 APT/PIP/ROS 源为清华大学镜像
- Robotiq 夹爪最大速度从 `0.150 m/s` 改为 `1.0 m/s`（修改 `hardware_interface.cpp` 中的 `kGripperMaxSpeed`）
- `libfranka` v0.18.2, `franka_ros2` v2.1.0, `franka_description` v1.3.0
- `ros2_robotiq_gripper` commit `2ff85455d4b9f973c4b0bab1ce95fb09367f0d26`

---

## 常见问题

### 串口无法打开

```
SerialException: could not open port
```

- 检查设备是否正确连接，`ls /dev/serial/by-id/` 确认设备存在
- Docker 中运行时确保启动容器前设备已插入，且 docker-compose 已挂载 `/dev`
- 检查设备权限：`sudo chmod 666 /dev/serial/by-id/<设备名>`

### 机械臂 reflex 保护触发 (power_limit_violation / joint_velocity_violation)

```
[FrankaHardwareInterface]: libfranka: Move command aborted: motion aborted by reflex! ["power_limit_violation"]
[FrankaHardwareInterface]: libfranka: Move command aborted: motion aborted by reflex! ["joint_velocity_violation"]
```

- **确认 GELLO 已先于 Franka 控制器启动**，且正常发布关节状态
- 检查 GELLO 关节值是否正常：`ros2 topic echo /left/gello/joint_states`，不应出现 ±2.9007 等关节极限值
- 如果值异常，需重新校准 GELLO（步骤 2）或将 GELLO 物理摆到机械臂当前位姿附近
- GELLO 正常后，重启 Franka 控制器

### 机械臂运动不平滑

降低 USB 延迟（在宿主机执行）：

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### libfranka 版本不兼容

报错 `Incompatible library version` 时，检查机械臂系统版本与 `libfranka` 版本的兼容性：
https://frankarobotics.github.io/docs/compatibility.html

### Franka 逆运动学

Franka 官方配置的 MoveIt 逆运动学求解器为 LMA (Levenberg-Marquardt Algorithm)。

### TV 环境

UV 虚拟环境位于 `/workspace/.venv`，使用 `source /workspace/.venv/bin/activate` 激活。

---

## 硬件参考

- 机械臂：Franka FR3
- 主端控制器：GELLO (Dynamixel XL330-M288)
- 夹爪：Robotiq 2F-85
- 力传感器：Robotiq FT
- 相机：Intel RealSense D405 + D415
- 通信转换器：U2D2 (FTDI) 或 OpenRB-150
