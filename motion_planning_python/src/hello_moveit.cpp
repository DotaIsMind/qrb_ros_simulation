/**
 * @file hello_moveit.cpp
 * @brief A basic ROS 2 and MoveIt 2 program to control a robot arm with gripper
 *
 * This program demonstrates how to use ROS 2 and MoveIt 2 to control a robot arm.
 * It sets up a node, creates a MoveGroupInterface for the arm and gripper, sets a target pose for the gripper_base,
 * plans a trajectory, and executes the planned motion with gripper control.
 *
 * @author Addison Sears-Collins
 * @date December 15, 2024
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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

  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the robot.
  // The use of auto allows the compiler to automatically deduce the type of variable.
  // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "rm_group_controller");
  auto gripper_group_interface = MoveGroupInterface(node, "hand_controller");

  // Specify a planning pipeline to be used for further planning
  arm_group_interface.setPlanningPipelineId("ompl");

  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);

  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);

  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Function to control gripper using SRDF named states
  auto control_gripper = [&gripper_group_interface, &logger](const std::string& state_name) {
    gripper_group_interface.setNamedTarget(state_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (gripper_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      gripper_group_interface.execute(plan);
      RCLCPP_INFO(logger, "Gripper moved to state: %s", state_name.c_str());
    } else {
      RCLCPP_ERROR(logger, "Failed to move gripper to state: %s", state_name.c_str());
    }
  };

  // Step 1: Open gripper initially
  RCLCPP_INFO(logger, "Step 1: Opening gripper initially");
  control_gripper("open");
  // Wait a moment for gripper to open
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Step 2: Move to target position
  RCLCPP_INFO(logger, "Step 2: Moving to target position");
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    // msg.header.frame_id = "arm_base_link";
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = -0.5;
    msg.pose.position.y = 0.5;
    msg.pose.position.z = 0.2;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose);

  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
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
    arm_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Arm moved to target position successfully");
  }
  else
  {
    RCLCPP_ERROR(logger, "Arm planning failed!");
  }

  // Wait a moment for arm to reach position
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Step 3: Close gripper to grasp object
  RCLCPP_INFO(logger, "Step 3: Closing gripper to grasp object");
  control_gripper("close");
  // Wait a moment for gripper to close
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Step 4: Move to a different position with grasped object
  RCLCPP_INFO(logger, "Step 4: Moving to different position with grasped object");
  auto const arm_target_pose2 = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "arm_base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.5;
    msg.pose.position.z = 0.5;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose2);

  auto const [success2, plan2] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success2)
  {
    arm_group_interface.execute(plan2);
    RCLCPP_INFO(logger, "Arm moved to new position with grasped object");
  }
  else
  {
    RCLCPP_ERROR(logger, "Arm planning failed!");
  }

  // Wait a moment
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Step 5: Open gripper to release object
  RCLCPP_INFO(logger, "Step 5: Opening gripper to release object");
  control_gripper("open");

  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}