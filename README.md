# -
# UAV Formation Inspection Simulation
# 面向巡检任务的无人机编队策略研究

## 项目概述

本项目为无人机编队巡检仿真系统，基于PX4飞控、ROS2 Humble、Gazebo Garden构建。主要研究面向输电线路巡检任务的无人机编队策略。

### 仿真环境特性

- **山脉地形**: 高度约50m，长度约200m的弯曲不规则山体（OBJ网格模型）
- **输电设施**:
  - 山脚输电塔（高度20m，地形高度4m）
  - 山顶输电塔（高度20m，地形高度56.6m）
  - 两塔之间的下垂弯曲电线（巡检目标）
- **障碍物**: 20棵树木（高度5-10m），分布在输电线周围
- **无人机编队**: 3架X500四旋翼无人机，采用Leader-Follower编队模式

## 目录结构

```
uav_inspection/
├── worlds/                       # Gazebo世界文件
│   ├── inspection_world.sdf      # 主仿真世界文件
│   ├── mountain_mesh.obj         # 山脉网格模型
│   ├── mountain_mesh_material.mtl# 材质文件
│   └── world_config.json         # 世界配置信息
├── launch/                       # ROS2启动文件
├── config/                       # 配置文件
├── scripts/                      # 脚本文件
│   ├── generate_world.py         # 世界生成脚本
│   └── start_simulation.sh       # 一键启动脚本
├── package.xml
├── CMakeLists.txt
└── README.md
```

## 环境要求

### 软件依赖

| 软件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 22.04 LTS | 操作系统 |
| ROS2 | Humble | 机器人操作系统 |
| Gazebo | Garden | 仿真环境 |
| PX4 | v1.14+ | 飞控固件 |
| MicroXRCEAgent | latest | DDS通信桥接 |
| QGroundControl | latest | 地面站 |

### 系统配置

确保以下环境变量已设置：

```bash
# ROS2
source /opt/ros/humble/setup.bash

# PX4
export PX4_DIR=$HOME/PX4-Autopilot

# Gazebo资源路径
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/environment-main/uav_inspection/worlds:$GZ_SIM_RESOURCE_PATH
```

## 安装步骤

### 1. 构建PX4（如果尚未构建）

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### 2. 安装Python依赖

```bash
sudo apt install -y python3-pip python3-numpy python3-pil python3-scipy
```

## 快速启动

### 方法一：一键启动脚本（推荐）

```bash
cd ~/ros2_ws/src/environment-main/uav_inspection/scripts
chmod +x start_simulation.sh
./start_simulation.sh
```

脚本会自动打开5个终端窗口：
1. **Gazebo** - 仿真世界
2. **MicroXRCE-DDS Agent** - ROS2与PX4通信桥接
3. **PX4 UAV1** - 领航无人机
4. **PX4 UAV2** - 跟随无人机1
5. **PX4 UAV3** - 跟随无人机2

**注意**: QGroundControl需要手动启动。

### 方法二：手动分步启动

#### 步骤1: 启动Gazebo仿真世界（终端1）

```bash
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/environment-main/uav_inspection/worlds:~/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH
gz sim -v3 -r ~/ros2_ws/src/environment-main/uav_inspection/worlds/inspection_world.sdf
```

**注意**: 
- `-r` 参数让仿真自动开始播放
- 必须包含PX4模型路径，否则无人机无法加载

#### 步骤2: 启动MicroXRCE-DDS Agent（终端2）

```bash
MicroXRCEAgent udp4 -p 8888
```

#### 步骤3: 启动三架无人机（终端3-5）

**UAV1 (Leader):**
```bash
cd ~/PX4-Autopilot
source ./build/px4_sitl_default/rootfs/gz_env.sh
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=power_line_inspection \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="-146.32,-85.89,12.0,0,0,1.57" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1
```

**UAV2 (Follower 1):**
```bash
cd ~/PX4-Autopilot
source ./build/px4_sitl_default/rootfs/gz_env.sh
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=power_line_inspection \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="-146.32,-82.89,12.0,0,0,1.57" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 2
```

**UAV3 (Follower 2):**
```bash
cd ~/PX4-Autopilot
source ./build/px4_sitl_default/rootfs/gz_env.sh
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=power_line_inspection \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="-146.32,-79.89,12.0,0,0,1.57" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 3
```

#### 步骤4: 启动QGroundControl

```bash
QGroundControl
```

## 世界配置信息

### 无人机生成位置

无人机生成在基站塔西南侧的平坦区域（坡度<0.02），确保初始姿态水平稳定。

| 无人机 | X坐标 | Y坐标 | Z坐标 | 角色 | 地形高度 | 坡度 |
|--------|-------|-------|-------|------|----------|------|
| UAV1 | -146.32 | -85.89 | 12.0 | Leader | 0.09m | <0.02 |
| UAV2 | -146.32 | -82.89 | 12.0 | Follower 1 | 0.09m | <0.02 |
| UAV3 | -146.32 | -79.89 | 12.0 | Follower 2 | 0.09m | <0.02 |

**注**：无人机统一 `yaw=1.57`（机头朝线路方向）；初始高度 12.0m 提供充足地形间隙；生成位置经过坡度筛选，避免山坡地形导致初始姿态倾斜，从而解决 `Attitude failure` 解锁失败问题。

### 输电塔位置

| 输电塔 | X坐标 | Y坐标 | 地形高度 | 塔高度 | 塔顶高度 |
|--------|-------|-------|----------|--------|----------|
| 山脚塔 | -116.32 | -52.89 | 4.00m | 20m | 24.00m |
| 山顶塔 | -19.29 | 23.57 | 56.60m | 20m | 76.60m |

### 电线高度

| 位置 | 电线顶端高度 |
|------|--------------|
| 山脚塔电线 | 22.30m |
| 山顶塔电线 | 74.90m |

### 巡检目标高度

无人机应在电线上方约5m处飞行：
- 山脚区域: z ≈ 27.30m
- 山顶区域: z ≈ 79.90m

## 重新生成世界文件

如果需要修改地形参数，可以编辑 `scripts/generate_world.py` 中的配置：

```python
WORLD_SIZE   = 400          # 世界XY范围 (m)
WORLD_Z_MAX  = 60.0         # 地形最大高度 (m)
TOWER_H      = 20.0         # 输电塔高度 (m)
N_TREES      = 20           # 树木数量
```

然后运行：

```bash
cd ~/ros2_ws/src/environment-main/uav_inspection/scripts
python3 generate_world.py
```

## ROS2话题说明

### 发布话题（每架无人机）

| 话题 | 消息类型 | 频率 | 说明 |
|------|----------|------|------|
| /fmu/out/vehicle_odometry | px4_msgs/VehicleOdometry | 50Hz | 里程计 |
| /fmu/out/vehicle_local_position | px4_msgs/VehicleLocalPosition | 50Hz | 本地位置 |
| /fmu/out/vehicle_global_position | px4_msgs/VehicleGlobalPosition | 10Hz | 全局位置 |

### 订阅话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| /fmu/in/trajectory_setpoint | px4_msgs/TrajectorySetpoint | 轨迹设定点 |
| /fmu/in/offboard_control_mode | px4_msgs/OffboardControlMode | Offboard模式 |
| /fmu/in/vehicle_command | px4_msgs/VehicleCommand | 车辆命令 |

## 下一步开发

1. **航迹规划模块**: 基于人工蜂群算法的航迹规划
2. **编队控制**: Leader-Follower编队控制与一致性协议
3. **避障系统**: 基于三维栅格地图的障碍物检测与规避
4. **RBF神经网络**: PID参数自适应调整

## 关键环境变量说明

| 环境变量 | 说明 |
|----------|------|
| `GZ_SIM_RESOURCE_PATH` | Gazebo资源路径，必须包含世界文件和PX4模型路径 |
| `PX4_GZ_STANDALONE=1` | 启用独立模式，PX4连接到已运行的Gazebo实例 |
| `PX4_GZ_WORLD=power_line_inspection` | 指定Gazebo世界名称（必须与SDF中的world name一致） |
| `PX4_GZ_MODEL_POSE` | 无人机生成位置 (x,y,z,roll,pitch,yaw) |
| `PX4_SIM_MODEL=gz_x500` | 无人机模型类型 |
| `source gz_env.sh` | 加载PX4的Gazebo环境配置（必须） |
| `-r` (gz sim参数) | 仿真自动开始播放 |

## 常见问题

### Q: PX4报错 "Timed out waiting for Gazebo world"

确保：
1. Gazebo已启动并正在运行（使用 `-r` 参数自动开始仿真）
2. 设置了 `PX4_GZ_WORLD=power_line_inspection` 环境变量
3. 世界名称与SDF文件中的 `<world name="power_line_inspection">` 一致

### Q: QGC无法连接无人机

检查MicroXRCEAgent是否正在运行，并且端口8888未被占用。

### Q: QGC里显示 "Arming denied" 或无法解锁

如果提示包含以下内容：
- `Attitude failure detected`（姿态失败，通常由出生点与地形碰撞引起）
- `No manual control input`（无手动输入）

请按下面顺序处理：
1. 确认使用平坦区域生成点（`x=-146.32, y=-85.89~-79.89, z=12.0, yaw=1.57`），避免山坡地形导致姿态超限。
2. 在 QGC 的 **Parameters** 中设置 `COM_RC_IN_MODE=4`（禁用遥控器依赖），然后重启对应 PX4 实例。
3. 确认 Gazebo 以 `-r` 启动且仿真未暂停，再尝试解锁。

### Q: 无人机不显示在Gazebo中

1. 确保使用 `PX4_GZ_STANDALONE=1` 参数启动PX4
2. 确保设置了 `PX4_GZ_WORLD=power_line_inspection` 环境变量
3. 确保Gazebo先于PX4启动，并使用 `-r` 参数开始仿真
4. 等待PX4完成初始化（约10-15秒）

### Q: 电线穿模进入山体

已修复：输电塔高度增加到20m，电线下垂系数降低到5%。

### Q: QGC无法解锁，报错 "Attitude failure (roll)" — 姿态超限

无人机生成在具有坡度的地形上，导致机体倾斜角超过预检阈值，PX4 认为姿态异常拒绝解锁。

**解决方案**：
1. 使用本README文档中最新平坦区域生成点（`x=-146.32, y=-85.89~-79.89, z=12.0, yaw=1.57`）。该位置经过坡度筛选（坡度<0.02），确保初始姿态水平稳定。
2. 如需自定义位置，可用 `scripts/generate_world.py` 中的 `find_flat_spot()` 函数自动搜索平坦地形。

### Q: QGC无法解锁，报错 "No manual control input" — 无遥控器输入

SITL 模式下没有连接 RC 遥控器/手柄，PX4 默认要求有遥控输入才能解锁。

**解决方案**：
1. 在启动 PX4 前设置参数禁用 RC 检查：
   ```bash
   param set COM_RC_IN_MODE 0
   ```
2. 或在启动命令中直接传递参数：
   ```bash
   ./build/px4_sitl_default/bin/px4 -i 1 -p "COM_RC_IN_MODE=0"
   ```
3. 也可以设置遥控器丢失例外：
   ```bash
   param set COM_RCL_EXCEPT 4
   ```

## 许可证

MIT License

## 作者

tjc - 毕业设计项目

## 更新（第二部分已落地）

已在 `uav_inspection` 中新增三维栅格地图构建、改进人工蜂群路径规划（含B样条平滑）、以及ROS2地图/路径发布节点，可直接作为你毕设“第二阶段”代码基础。
