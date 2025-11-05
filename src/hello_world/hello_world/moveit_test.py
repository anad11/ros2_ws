#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander

def main(args=None):
    rclpy.init(args=args)
    node = Node('moveit_test_node')

    # Connect to a MoveIt group (replace 'arm' with your MoveIt group name)
    arm_group = MoveGroupCommander('arm')

    # Print current pose
    node.get_logger().info(f'Current Pose: {arm_group.get_current_pose().pose}')

    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


