#!/bin/bash
# Performance monitoring script for ROS2 simulation

echo "=== ROS2仿真性能监控 ==="
echo "时间: $(date)"
echo ""

# Check system resources
echo "=== 系统资源 ==="
echo "CPU使用率:"
top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1
echo "内存使用率:"
free -h | grep Mem | awk '{print $3"/"$2 " (" $3/$2*100 "%)"}'
echo ""

# Check ROS2 processes
echo "=== ROS2进程状态 ==="
echo "控制器管理器进程:"
ps aux | grep controller_manager | grep -v grep || echo "未找到控制器管理器进程"
echo ""

echo "Gazebo进程:"
ps aux | grep gazebo | grep -v grep || echo "未找到Gazebo进程"
echo ""

# Check ROS2 topics
echo "=== ROS2话题状态 ==="
echo "活跃的ROS2话题:"
ros2 topic list 2>/dev/null | head -10 || echo "无法获取ROS2话题列表"
echo ""

# Check controller status
echo "=== 控制器状态 ==="
echo "控制器列表:"
ros2 control list_controllers 2>/dev/null || echo "无法获取控制器列表"
echo ""

# Check system load
echo "=== 系统负载 ==="
uptime
echo ""

# Check disk usage
echo "=== 磁盘使用情况 ==="
df -h | grep -E "(/$|/home)"
echo ""

echo "=== 性能建议 ==="
echo "1. 如果CPU使用率持续高于80%，考虑降低控制器频率"
echo "2. 如果内存使用率高于90%，考虑关闭不必要的进程"
echo "3. 如果出现overrun警告，检查系统负载和控制器配置"
echo "4. 确保Gazebo世界文件使用正确的物理引擎(bullet而不是ignored)"
echo ""

echo "监控完成。按Ctrl+C退出。"
echo "每30秒自动刷新..."

# Continuous monitoring
while true; do
    sleep 30
    clear
    echo "=== ROS2仿真性能监控 (自动刷新) ==="
    echo "时间: $(date)"
    echo ""
    
    # Quick system check
    echo "系统负载: $(uptime | awk -F'load average:' '{print $2}')"
    echo "内存使用: $(free -h | grep Mem | awk '{print $3"/"$2}')"
    echo ""
    
    # Check for overrun warnings
    echo "最近的过载警告:"
    journalctl --since "5 minutes ago" | grep -i "overrun\|missed" | tail -3 || echo "未发现过载警告"
    echo ""
done 