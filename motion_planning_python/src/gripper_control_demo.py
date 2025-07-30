#!/usr/bin/env python3
"""
Gripper Control Demo for RML 63 Robot Arm

This script demonstrates how to control the robot arm and gripper to pick and place objects.
It includes proper gripper control to ensure objects are grasped and moved correctly.

Author: Assistant
Date: 2024
"""

import rclpy
from rclpy.node import Node
from moveit.planning_interface import MoveGroupInterface
from geometry_msgs.msg import PoseStamped
import time


class GripperControlDemo(Node):
    def __init__(self):
        super().__init__('gripper_control_demo')
        
        # Create MoveGroup interfaces
        self.arm_group = MoveGroupInterface(self, "rm_group_controller")
        self.gripper_group = MoveGroupInterface(self, "hand_controller")
        
        # Set planning parameters
        self.arm_group.set_planning_pipeline_id("ompl")
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        
        self.get_logger().info("Gripper Control Demo initialized")
    
    def control_gripper(self, open_gripper=True):
        """
        Control the gripper to open or close
        
        Args:
            open_gripper (bool): True to open gripper, False to close
        """
        if open_gripper:
            self.get_logger().info("Opening gripper...")
            # Open gripper - set joints to open position
            target_joint_values = [0.0, 0.0, 0.0, 0.0]
        else:
            self.get_logger().info("Closing gripper...")
            # Close gripper - set joints to closed position
            target_joint_values = [1.0, -1.0, -1.0, 1.0]
        
        self.gripper_group.set_joint_value_target(target_joint_values)
        
        # Plan and execute
        plan = self.gripper_group.plan()
        if plan[0]:
            self.gripper_group.execute(plan[1])
            self.get_logger().info(f"Gripper {'opened' if open_gripper else 'closed'} successfully")
            return True
        else:
            self.get_logger().error("Gripper planning failed!")
            return False
    
    def move_arm_to_pose(self, x, y, z, orientation=None):
        """
        Move the arm to a specific pose
        
        Args:
            x, y, z (float): Position coordinates
            orientation (list): Orientation as [x, y, z, w]
        """
        pose = PoseStamped()
        pose.header.frame_id = "arm_base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        if orientation:
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]
        else:
            # Default orientation
            pose.pose.orientation.x = 1.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
        
        self.arm_group.set_pose_target(pose)
        
        plan = self.arm_group.plan()
        if plan[0]:
            self.arm_group.execute(plan[1])
            self.get_logger().info(f"Arm moved to position ({x}, {y}, {z}) successfully")
            return True
        else:
            self.get_logger().error("Arm planning failed!")
            return False
    
    def run_pick_and_place_demo(self):
        """
        Run a complete pick and place demonstration
        """
        self.get_logger().info("Starting Pick and Place Demo")
        
        # Step 1: Open gripper initially
        self.get_logger().info("Step 1: Opening gripper initially")
        if not self.control_gripper(open_gripper=True):
            return False
        time.sleep(2)
        
        # Step 2: Move to object position
        self.get_logger().info("Step 2: Moving to object position")
        if not self.move_arm_to_pose(0.1, 0.05, 0.05):
            return False
        time.sleep(2)
        
        # Step 3: Close gripper to grasp object
        self.get_logger().info("Step 3: Closing gripper to grasp object")
        if not self.control_gripper(open_gripper=False):
            return False
        time.sleep(2)
        
        # Step 4: Move to target position with grasped object
        self.get_logger().info("Step 4: Moving to target position with grasped object")
        if not self.move_arm_to_pose(0.2, 0.1, 0.1):
            return False
        time.sleep(2)
        
        # Step 5: Open gripper to release object
        self.get_logger().info("Step 5: Opening gripper to release object")
        if not self.control_gripper(open_gripper=True):
            return False
        time.sleep(2)
        
        # Step 6: Move back to home position
        self.get_logger().info("Step 6: Moving back to home position")
        if not self.move_arm_to_pose(0.0, 0.0, 0.3):
            return False
        
        self.get_logger().info("Pick and Place Demo completed successfully!")
        return True


def main(args=None):
    rclpy.init(args=args)
    
    demo = GripperControlDemo()
    
    try:
        success = demo.run_pick_and_place_demo()
        if success:
            demo.get_logger().info("Demo completed successfully!")
        else:
            demo.get_logger().error("Demo failed!")
    except Exception as e:
        demo.get_logger().error(f"Demo failed with exception: {e}")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 