#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import time
import math

class MecanumTestNode(Node):
    def __init__(self):
        super().__init__('mecanum_test_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_position', 10)
        
        # Subscribers
        self.target_reached_sub = self.create_subscription(
            Bool, '/target_reached', self.target_reached_callback, 10)
        
        # Timer for test sequence
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.test_phase = 0
        self.start_time = self.get_clock().now()
        self.target_reached = False
        
        self.get_logger().info("Mecanum test node started")
    
    def target_reached_callback(self, msg):
        if msg.data:
            self.target_reached = True
            self.get_logger().info("Target reached!")
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.test_phase == 0:
            # Test 1: Move forward
            if elapsed < 3.0:
                cmd = Twist()
                cmd.linear.x = 0.2
                self.cmd_vel_pub.publish(cmd)
                if elapsed < 0.5:
                    self.get_logger().info("Test 1: Moving forward")
            else:
                self.test_phase = 1
                self.start_time = current_time
        
        elif self.test_phase == 1:
            # Test 2: Move sideways
            if elapsed < 3.0:
                cmd = Twist()
                cmd.linear.y = 0.2
                self.cmd_vel_pub.publish(cmd)
                if elapsed < 0.5:
                    self.get_logger().info("Test 2: Moving sideways")
            else:
                self.test_phase = 2
                self.start_time = current_time
        
        elif self.test_phase == 2:
            # Test 3: Rotate
            if elapsed < 3.0:
                cmd = Twist()
                cmd.angular.z = 0.5
                self.cmd_vel_pub.publish(cmd)
                if elapsed < 0.5:
                    self.get_logger().info("Test 3: Rotating")
            else:
                self.test_phase = 3
                self.start_time = current_time
        
        elif self.test_phase == 3:
            # Test 4: Stop
            if elapsed < 2.0:
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                if elapsed < 0.5:
                    self.get_logger().info("Test 4: Stopping")
            else:
                self.test_phase = 4
                self.start_time = current_time
        
        elif self.test_phase == 4:
            # Test 5: Position control
            if elapsed < 1.0:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "odom"
                pose.pose.position.x = 1.0
                pose.pose.position.y = 0.5
                pose.pose.orientation.w = 1.0
                self.target_pose_pub.publish(pose)
                if elapsed < 0.5:
                    self.get_logger().info("Test 5: Position control to (1.0, 0.5)")
            elif elapsed < 15.0:
                # Wait for target to be reached
                pass
            else:
                self.test_phase = 5
        
        elif self.test_phase == 5:
            # Tests complete
            self.get_logger().info("All tests completed!")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MecanumTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()