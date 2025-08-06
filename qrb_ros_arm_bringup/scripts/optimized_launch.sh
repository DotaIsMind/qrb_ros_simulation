#!/bin/bash
# Optimized launch script for RML_63 with Gazebo and MoveIt 2

cleanup() {
    echo "正在清理..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

# System optimization for ROS2 simulation
echo "正在优化系统性能..."

# Set ROS2 environment variables for better performance
export ROS_DOMAIN_ID=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCL_LOGGING_USE_STDOUT=1
export RCL_LOGGING_BUFFERED_STREAM=1

# Optimize system parameters if possible
if command -v cpupower &> /dev/null; then
    echo "设置CPU性能模式..."
    sudo cpupower frequency-set -g performance 2>/dev/null || true
fi

# Increase system limits for ROS2
echo "调整系统限制..."
sudo sysctl -w kernel.sched_rt_runtime_us=-1 2>/dev/null || true
sudo sysctl -w kernel.sched_rt_period_us=1000000 2>/dev/null || true

# Optimize memory management
echo "优化内存管理..."
sudo sysctl -w vm.swappiness=10 2>/dev/null || true
sudo sysctl -w vm.dirty_ratio=15 2>/dev/null || true
sudo sysctl -w vm.dirty_background_ratio=5 2>/dev/null || true

echo "启动Gazebo仿真..."
ros2 launch qrb_ros_sim_gazebo gazebo_rml_63_gripper.launch.py &

# Wait for Gazebo to fully initialize
echo "等待Gazebo完全启动..."
sleep 30

echo "启动MoveIt配置..."
ros2 launch qrb_ros_arm_moveit_config demo.launch.py &

echo "所有组件已启动。按Ctrl+C停止仿真。"
echo "注意：控制器频率已优化为50Hz以减少过载警告。"

# Keep the script running until Ctrl+C
wait 