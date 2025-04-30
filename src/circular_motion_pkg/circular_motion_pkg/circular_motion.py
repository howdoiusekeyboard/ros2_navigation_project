#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircularMotionController(Node):
    def __init__(self):
        super().__init__('circular_motion_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_in_circle)
        self.get_logger().info('Circular motion controller has been started')
        
        # Circle parameters
        self.linear_speed = 2.0  # Units per second
        self.angular_speed = 1.0  # Radians per second
        
    def move_in_circle(self):
        # Create a Twist message
        twist = Twist()
        
        # Set linear velocity
        twist.linear.x = self.linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        # Set angular velocity
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_speed
        
        # Publish the message
        self.publisher.publish(twist)
        self.get_logger().info('Publishing: Linear velocity = %f, Angular velocity = %f' 
                              % (self.linear_speed, self.angular_speed))

def main(args=None):
    rclpy.init(args=args)
    controller = CircularMotionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Stop the turtle before shutting down
        stop_msg = Twist()
        controller.publisher.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
