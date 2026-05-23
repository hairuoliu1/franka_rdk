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
│   ├── policies/                        # 策略模型 (OpenPI)
│   ├── ros_bridge.py                    # Python 3.10 ROS 2 bridge 子进程
│   └── ros_bridge_client.py             # LeRobot 主进程读取 bridge 输出
├── test/                                # 测试与数据采集脚本
│   ├── test_recording.py                # 双臂数据采集端到端测试
│   ├── test_action_listener.py          # 监听 ROS action
│   ├── fake_robot_action_publisher.py   # 模拟发送 action
│   ├── test_obs_listener.py             # 监听 ROS observation
│   └── fake_robot_state_publisher.py    # 模拟发送 observation
└── lerobot/                             # LeRobot 框架 (git submodule)
```

> **lerobot 为 git submodule**，指向上游 [huggingface/lerobot](https://github.com/huggingface/lerobot)。当前仓库直接按子模块指针使用，不再需要额外应用本地 patch。

---

## Git 仓库与子模块管理

### 克隆

```bash
git clone --recursive <repo-url>
# 如果已 clone 但忘记 --recursive：
git submodule update --init
```

### 更新子模块

```bash
git submodule update --init --recursive lerobot
cd lerobot
```

如果需要升级 LeRobot，先在子模块内切换到目标 commit，再回到主仓库提交子模块指针：

```bash
cd lerobot
git fetch
git checkout <target-commit-or-tag>
cd ..
git add lerobot
git commit -m "update lerobot submodule"
```

---

## 配置文件

所有配置文件集中存放在 `/workspace/src/config/`：

| 文件 | 用途 |
|---|---|
| `fr3_config.yaml` | FR3 控制器配置 (IP、namespace 等) |
| `gello_publisher.yaml` | GELLO 配置 (端口、偏移量、关节方向) |
| `robotiq_gripper_config.yaml` | Robotiq 夹爪配置 (端口、namespace) |

### fr3_config.yaml 双臂示例

```yaml
LEFT:
  arm_id: "fr3"
  arm_prefix: "left"
  fake_sensor_commands: "false"
  joint_sources: ["joint_states", "franka_gripper/joint_states"]
  joint_state_rate: 30
  load_gripper: "true"
  namespace: "left"
  robot_ip: "172.16.0.2"
  urdf_file: "fr3/fr3.urdf.xacro"
  use_fake_hardware: "false"
  use_rviz: "false"

RIGHT:
  arm_id: "fr3"
  arm_prefix: "right"
  fake_sensor_commands: "false"
  joint_sources: ["joint_states", "franka_gripper/joint_states"]
  joint_state_rate: 30
  load_gripper: "true"
  namespace: "right"
  robot_ip: "172.16.0.3"
  urdf_file: "fr3/fr3.urdf.xacro"
  use_fake_hardware: "false"
  use_rviz: "false"
```

### gello_publisher.yaml 双臂示例

```yaml
LEFT:
  namespace: "left"
  com_port: "usb-FTDI_USB__-__Serial_Converter_FTAWWGWP-if00-port0"
  num_arm_joints: 7
  joint_signs: [1, 1, 1, -1, 1, -1, 1]
  gripper: true
  assembly_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  gripper_range_rad: [2.307, 3.527]

RIGHT:
  namespace: "right"
  com_port: "usb-FTDI_USB__-__Serial_Converter_FTAWANP9-if00-port0"
  num_arm_joints: 7
  joint_signs: [1, -1, 1, -1, 1, -1, 1]
  gripper: true
  assembly_offsets: [3.142, 0.0, 3.142, 4.712, 3.142, 1.571, 0.0]
  gripper_range_rad: [3.914, 5.134]
```

---

## Python 环境说明

ROS 2 Humble 默认使用 Python 3.10。当前数据采集链路也按 Python 3.10 运行：主进程加载 LeRobot，本地 `src/ros_bridge.py` 会用 `/usr/bin/python3.10` 作为 ROS bridge 子进程读取 ROS 2 话题。

确认环境：

```bash
cd /home/franka/franka_rdk
python --version
/usr/bin/python3.10 --version
```

LeRobot 子模块直接按仓库记录的 commit 使用。克隆或切换分支后，先运行 `git submodule update --init --recursive lerobot`，再确认当前 Python 环境可以 import `lerobot`。

常用命令：

```bash
# 查看当前 Python 环境中的包
python -m pip list

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

将输出的 `assembly_offsets`、`gripper_range_rad` 等值更新到 `gello_publisher.yaml`。`gripper_range_rad[0]` 映射到夹爪控制百分比 `0.0`，`gripper_range_rad[1]` 映射到 `1.0`。

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

Robotiq 2F-85 夹爪 raw 范围：`0.0` = 完全张开，`0.085` = 最大闭合。控制 topic `/gripper/gripper_client/target_gripper_width_percent` 保持开口百分比语义：`1.0` = 张开，`0.0` = 闭合；采集链路把 action/observation 的 `joint_positions_7` 记录为 raw Robotiq 绝对位置。

### 步骤 8：启动力传感器（可选）

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

GELLO (Dynamixel) ──→ gello_publisher ──→ /left/gello/joint_states
                                           /right/gello/joint_states
                                               │
                                               ▼
                         joint_impedance_controller ──→ left/right Franka FR3

                         gripper_manager ──→ /left/gripper/joint_states
                                             /right/gripper/joint_states

Robotiq FT Sensor ──→ /robotiq_force_torque_sensor_broadcaster/wrench
```

> 注意：启动全部节点后的话题列表会多出 `/left/gello/joint_states`、`/right/gello/joint_states`、`/left/franka/joint_states`、`/right/franka/joint_states` 等。

---

## 关节状态读取约定

**ROS 发布的 joint topic 中关节名称顺序不固定，切勿用 list 索引直接读取，必须通过字典查找。**

实际观察到的 `/left/franka/joint_states` / `/right/franka/joint_states` 顺序可能如下（注意 joint2 和 joint3 是反的）：

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

## 数据采集

### 正式采集入口

正式数据采集使用 `/home/franka/franka_rdk/src/lerobot_record.py`，推荐用模块方式启动：

```bash
cd /home/franka/franka_rdk
/home/franka/miniconda3/envs/lerobot/bin/python -m src.lerobot_record ...
```

采集脚本使用双臂封装：

- `BiFrankaFr3RobotiqGripper`：读取 left/right Franka state 和相机。
- `BiGelloRosLeader`：被动监听 left/right GELLO action，不向机器人重复下发指令。
- `src/ros_bridge.py`：由 Python 3.10 子进程读取 ROS 2 话题，并写入 `/dev/shm/lerobot_state_<namespace>.json`，避免 Python 3.12 环境直接 import `rclpy`。

`src` 中的单臂 `franka_fr3_robotiq_gripper` 和 `gello_ros_leader` 默认使用左臂 namespace `left`。双臂封装里左臂沿用这个默认值，右臂配置显式覆盖为 `right`。

默认检查的话题：

| 数据 | 默认话题 |
|---|---|
| 左臂状态 | `/left/franka/joint_states` |
| 右臂状态 | `/right/franka/joint_states` |
| 左臂 GELLO | `/left/gello/joint_states` |
| 右臂 GELLO | `/right/gello/joint_states` |
| 左/右夹爪控制百分比 | `/left/gripper/gripper_client/target_gripper_width_percent`、`/right/gripper/gripper_client/target_gripper_width_percent` |
| 左/右 GELLO 夹爪 raw 目标 | `/left/gello/gripper_position`、`/right/gello/gripper_position` |

夹爪维度是 `left_joint_positions_7` / `right_joint_positions_7`。数据集中 observation 和 action 都采集 raw Robotiq 值：`0.0=张开，0.085=最大闭合`。控制链路仍通过 `target_gripper_width_percent` 发送开口百分比，采集 bridge 优先读取 `/gello/gripper_position`；如果旧版 GELLO publisher 尚未发布 raw topic，则临时从百分比 topic 换算成 raw 值。

### 运行前准备

```bash
cd /home/franka/franka_rdk
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
export ROS_DOMAIN_ID=0
```

录制前确保 GELLO、Franka 控制器和 Robotiq 夹爪已按前文步骤启动。跨机器通信时，如果 multicast 发现不稳定，给 robot 配置传 `--robot.left_arm_config.remote_ip=<机器人侧IP>` 和 `--robot.right_arm_config.remote_ip=<机器人侧IP>`，脚本会临时写入 `CYCLONEDDS_URI`。

### 采集一段双臂数据

默认输出路径为 `$HF_LEROBOT_HOME/<repo_id>`；使用 `--dataset.root` 可指定本地目录。`src.lerobot_record` 是多 episode 交互式录制流程，按右方向键开始每个 episode，按 Esc 停止整个录制流程。

```bash
cd /home/franka/franka_rdk
/home/franka/miniconda3/envs/lerobot/bin/python -m src.lerobot_record \
  --robot.type=bi_franka_fr3_robotiq_gripper \
  --robot.left_arm_config.ros_domain_id=0 \
  --robot.right_arm_config.ros_domain_id=0 \
  --robot.left_arm_config.topic_namespace=left \
  --robot.right_arm_config.topic_namespace=right \
  --robot.left_arm_config.use_bridge=true \
  --robot.right_arm_config.use_bridge=true \
  --robot.left_arm_config.use_ft_sensor=false \
  --robot.right_arm_config.use_ft_sensor=false \
  --robot.left_arm_config.cameras='{left_camera: {type: opencv, index_or_path: "/dev/video0", width: 640, height: 480, fps: 30, warmup_s: 3, fourcc: "MJPG"}}' \
  --robot.right_arm_config.cameras='{middle_zed: {type: zed2, serial_number: "<ZED_SERIAL>", width: 672, height: 376, fps: 30, warmup_s: 3}, right_camera: {type: opencv, index_or_path: "/dev/video2", width: 640, height: 480, fps: 30, warmup_s: 3, fourcc: "MJPG"}}' \
  --teleop.type=bi_gello_ros_leader \
  --teleop.left_arm_config.ros_domain_id=0 \
  --teleop.right_arm_config.ros_domain_id=0 \
  --teleop.left_arm_config.topic_namespace=left \
  --teleop.right_arm_config.topic_namespace=right \
  --teleop.left_arm_config.use_bridge=true \
  --teleop.right_arm_config.use_bridge=true \
  --dataset.repo_id=local/bimanual_franka_recording \
  --dataset.num_episodes=5 \
  --dataset.single_task="bimanual franka recording" \
  --dataset.fps=30 \
  --dataset.episode_time_s=60 \
  --dataset.reset_time_s=0 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.vcodec=auto \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

### 相机配置

当前采集只使用三路相机：中间是 ZED，左右两侧是 OpenCV。相机 key 必须全局唯一：

```bash
--robot.left_arm_config.cameras='{left_camera: {type: opencv, index_or_path: "/dev/video0", width: 640, height: 480, fps: 30, warmup_s: 3, fourcc: "MJPG"}}'
--robot.right_arm_config.cameras='{middle_zed: {type: zed2, serial_number: "<ZED_SERIAL>", width: 672, height: 376, fps: 30, warmup_s: 3}, right_camera: {type: opencv, index_or_path: "/dev/video2", width: 640, height: 480, fps: 30, warmup_s: 3, fourcc: "MJPG"}}'
```

如果当前 Python 环境没有安装 ZED SDK 的 `pyzed` 模块，`type: zed2` 会在导入或连接时失败，需要先安装 ZED SDK 和 Python bindings。

常用参数：

| 参数 | 说明 |
|---|---|
| `--robot.left_arm_config.topic_namespace` / `--robot.right_arm_config.topic_namespace` | 单臂默认是 `left`；双臂采集时左臂为 `left`，右臂为 `right`，需要与 ROS 配置中的 namespace 一致。 |
| `--robot.*.ros_domain_id` / `--teleop.*.ros_domain_id` | 设置 ROS_DOMAIN_ID。 |
| `--robot.*.remote_ip` | 跨机器 DDS 显式 peer discovery fallback。 |
| `--dataset.num_episodes` | 采集 episode 数量。 |
| `--dataset.episode_time_s` | 每个 episode 录制时长。 |
| `--dataset.reset_time_s` | episode 之间的 reset 时长。 |
| `--dataset.streaming_encoding` | 实时编码视频，减少 `save_episode()` 等待时间。 |

### 可视化录制数据

```bash
/home/franka/miniconda3/envs/lerobot/bin/python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id local/bimanual_franka_recording \
  --episode-index 0 \
```

录制控制快捷键：

| 按键 | 作用 |
|------|------|
| **→ 右方向键** | 开始或提前结束当前循环 (`exit_early = True`) |
| **← 左方向键** | 结束当前循环并标记重录该 episode (`rerecord_episode = True`) |
| **Esc** | 停止整个录制流程 (`stop_recording = True`) |

### 测试脚本

`test/test_recording.py` 只用于端到端验证采集链路，不作为正式数据采集入口。它会录制一个短 episode，结束后重新加载数据集，检查 episode、frame 数量和视频文件是否完整。

只验证 state/action 管线、不录相机：

```bash
/home/franka/miniconda3/envs/lerobot/bin/python test/test_recording.py \
  --repo-id local/bimanual_franka_recording_no_camera \
  --no-cameras \
  --episode-s 3 \
  --fps 15
```

### Observation/Action 测试工具

`test/` 目录下的脚本用于单独验证 ROS 通信：

| 脚本 | 用途 |
|------|------|
| `test/test_action_listener.py` | 监听 GELLO action |
| `test/fake_robot_action_publisher.py` | 模拟发送 action |
| `test/test_obs_listener.py` | 监听 robot observation |
| `test/fake_robot_state_publisher.py` | 模拟发送 observation |

```bash
python test/test_action_listener.py
python test/test_obs_listener.py
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
    --dataset.num_episodes=2 \
    --dataset.single_task="Pick up the yellow block" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true \
    --play_sounds=false \
    --dataset.push_to_hub=false \
    --dataset.vcodec=h264 \
    --dataset.fps=30 \
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

# ===== 终端 5: 正式数据采集 =====
cd /home/franka/franka_rdk
/home/franka/miniconda3/envs/lerobot/bin/python -m src.lerobot_record \
    --robot.type=bi_franka_fr3_robotiq_gripper \
    --robot.left_arm_config.ros_domain_id=0 \
    --robot.right_arm_config.ros_domain_id=0 \
    --robot.left_arm_config.topic_namespace=left \
    --robot.right_arm_config.topic_namespace=right \
    --robot.left_arm_config.cameras='{left_camera: {type: opencv, index_or_path: "/dev/video0", width: 640, height: 480, fps: 30, warmup_s: 3, fourcc: "MJPG"}}' \
    --robot.right_arm_config.cameras='{middle_zed: {type: zed2, serial_number: "<ZED_SERIAL>", width: 672, height: 376, fps: 30, warmup_s: 3}, right_camera: {type: opencv, index_or_path: "/dev/video2", width: 640, height: 480, fps: 30, warmup_s: 3, fourcc: "MJPG"}}' \
    --teleop.type=bi_gello_ros_leader \
    --teleop.left_arm_config.ros_domain_id=0 \
    --teleop.right_arm_config.ros_domain_id=0 \
    --teleop.left_arm_config.topic_namespace=left \
    --teleop.right_arm_config.topic_namespace=right \
    --dataset.repo_id=local/bimanual_franka_recording \
    --dataset.num_episodes=5 \
    --dataset.single_task="bimanual franka recording" \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=0 \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --dataset.vcodec=auto \
    --dataset.push_to_hub=false \
    --display_data=false \
    --play_sounds=false
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

---

## 硬件参考

- 机械臂：Franka FR3
- 主端控制器：GELLO (Dynamixel XL330-M288)
- 夹爪：Robotiq 2F-85
- 力传感器：Robotiq FT
- 相机：Intel RealSense D405 + D415
- 通信转换器：U2D2 (FTDI) 或 OpenRB-150
