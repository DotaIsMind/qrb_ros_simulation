#!/bin/bash
# System optimization script for ROS2 simulation

echo "Optimizing system for ROS2 simulation..."

# Set CPU governor to performance mode
if command -v cpupower &> /dev/null; then
    echo "Setting CPU governor to performance mode..."
    sudo cpupower frequency-set -g performance
fi

# Increase system limits for ROS2
echo "Increasing system limits..."
sudo sysctl -w kernel.sched_rt_runtime_us=-1
sudo sysctl -w kernel.sched_rt_period_us=1000000

# Set process priority
echo "Setting process priority..."
sudo renice -n -10 -p $$

# Optimize memory management
echo "Optimizing memory management..."
sudo sysctl -w vm.swappiness=10
sudo sysctl -w vm.dirty_ratio=15
sudo sysctl -w vm.dirty_background_ratio=5

# Set ROS2 environment variables for better performance
export ROS_DOMAIN_ID=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# Disable some ROS2 features that consume resources
export RCL_LOGGING_USE_STDOUT=1
export RCL_LOGGING_BUFFERED_STREAM=1

echo "System optimization complete!"
echo "You can now run your ROS2 simulation with reduced overrun warnings." 