# QRB ROS Arm Pick Place

这个ROS2包实现了机器人手臂的抓取和放置操作，使用MoveIt进行运动规划。

## 功能特性

- 自动抓取和放置物体
- 使用MoveIt进行运动规划
- 支持夹爪控制（打开/关闭）
- 完整的动作序列执行
- 错误处理和日志记录

## 依赖项

- ROS2 Humble
- MoveIt2
- Gazebo仿真环境
- 机器人手臂配置（SRDF文件）

## 安装

1. 将包克隆到您的工作空间：
```bash
cd ~/qrb_ros_simulation/src
git clone <repository_url> qrb_ros_arm_pick_place
```

2. 构建包：
```bash
cd ~/qrb_ros_simulation
colcon build --packages-select qrb_ros_arm_pick_place
```

3. 设置环境：
```bash
source install/setup.bash
```

## 使用方法

### 1. 启动仿真环境

首先启动Gazebo仿真和MoveIt：

```bash
ros2 launch qrb_ros_sim_gazebo gazebo.launch.py
```

在另一个终端中启动MoveIt：

```bash
ros2 launch qrb_ros_arm_moveit_config demo.launch.py
```

### 2. 运行抓取放置程序

使用launch文件启动：

```bash
ros2 launch qrb_ros_arm_pick_place pick_place.launch.py
```

或者直接运行可执行文件：

```bash
ros2 run qrb_ros_arm_pick_place pick_place
```

## 动作序列

程序会执行以下动作序列：

1. 移动到抓取位置上方0.2米处
2. 打开夹爪
3. 移动到抓取位置
4. 关闭夹爪夹取物体
5. 移动到放置位置上方
6. 向下移动到放置位置
7. 松开夹爪放置物体
8. 回到安全位置

## 配置

### 夹爪配置

夹爪使用SRDF文件中定义的预设状态：

- **open状态**：所有关节值为0
- **close状态**：
  - `joint01`: 0.5
  - `joint11`: 0
  - `joint02`: -0.5
  - `joint22`: 0

### 目标位置

程序使用以下目标位置：

- **抓取位置**：`(1.0, 0.0, 1.02)`
- **放置位置**：`(1.0, 0.5, 1.02)`
- **安全高度**：在目标位置上方0.2米

## 故障排除

### 常见问题

1. **规划失败**：检查机器人是否在有效配置范围内
2. **夹爪控制失败**：确认SRDF文件中的夹爪配置正确
3. **碰撞检测**：确保目标位置没有障碍物

### 调试

启用详细日志：

```bash
ros2 run qrb_ros_arm_pick_place pick_place --ros-args --log-level debug
```

## 许可证

BSD-3-Clause 