#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <chrono>
#include <thread>

/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or not.
 */
int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
    
  // Creates a node named "qrb_ros_arm_pick_place". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "qrb_ros_arm_pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  
  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("qrb_ros_arm_pick_place");
  
  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the robot.
  // The use of auto allows the compiler to automatically deduce the type of variable.
  // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "rm_group_controller");
  
  // Specify a planning pipeline to be used for further planning 
  arm_group_interface.setPlanningPipelineId("ompl");
    
  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");  
  
  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(5.0); // 增加规划时间
    
  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(0.3); // 降低速度
    
  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(0.3); // 降低加速度

  // 设置规划尝试次数
  arm_group_interface.setNumPlanningAttempts(10);
  
  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // 等待一下让MoveIt完全初始化
  RCLCPP_INFO(logger, "Waiting for MoveIt to initialize...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 打印当前机器人状态
  auto current_pose = arm_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current robot pose - Position: (%.3f, %.3f, %.3f)", 
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  RCLCPP_INFO(logger, "Current robot pose - Orientation: (%.3f, %.3f, %.3f, %.3f)", 
              current_pose.pose.orientation.x, current_pose.pose.orientation.y, 
              current_pose.pose.orientation.z, current_pose.pose.orientation.w);
  
  // Set a target pose for the end effector of the arm - 使用更保守的位置
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "arm_base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = -0.5;  // 减小距离
    msg.pose.position.y = 0.5;
    msg.pose.position.z = 0.0;  // 降低高度
    msg.pose.orientation.x = 1.0;  // 修正姿态
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();

  RCLCPP_INFO(logger, "Setting target pose...");
  arm_group_interface.setPoseTarget(arm_target_pose);
  
  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
  RCLCPP_INFO(logger, "Planning to target pose...");
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  // Try to execute the movement plan if it was created successfully
  // If the plan wasn't successful, report an error
  // Execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful! Executing plan...");
    auto result = arm_group_interface.execute(plan);
    if (result.val == 1) { // SUCCESS
      RCLCPP_INFO(logger, "Movement completed successfully!");
    } else {
      RCLCPP_ERROR(logger, "Movement execution failed with error code: %d", result.val);
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    RCLCPP_INFO(logger, "Trying to get current joint values...");
    
    // 尝试获取当前关节值
    auto current_state = arm_group_interface.getCurrentState();
    auto joint_names = arm_group_interface.getJointNames();
    auto joint_values = arm_group_interface.getCurrentJointValues();
    
    RCLCPP_INFO(logger, "Current joint values:");
    for (size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(logger, "  %s: %.3f", joint_names[i].c_str(), joint_values[i]);
    }
  }
    
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}