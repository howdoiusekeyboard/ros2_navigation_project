#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

class CircularMotionBridge(Node):
    def __init__(self):
        """
        Initialize the CircularMotionBridge ROS 2 node, setting up publishers, subscribers, and timers for controlling turtle motion.
        
        This constructor creates publishers and subscribers for velocity commands and motion parameters, initializes internal state variables, and starts a periodic timer to publish velocity updates.
        """
        super().__init__('circular_motion_bridge')
        
        # Publisher to control the turtle
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscribers for parameters
        self.linear_speed_subscriber = self.create_subscription(
            Float32,
            '/turtle1/linear_speed',
            self.linear_speed_callback,
            10
        )
        
        self.angular_speed_subscriber = self.create_subscription(
            Float32,
            '/turtle1/angular_speed',
            self.angular_speed_callback,
            10
        )
        
        self.motion_active_subscriber = self.create_subscription(
            Bool,
            '/turtle1/motion_active',
            self.motion_active_callback,
            10
        )
        
        # Parameters
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.motion_active = False
        
        # Timer for publishing at a regular rate
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info('Circular Motion Bridge has been started')
    
    def linear_speed_callback(self, msg):
        """
        Callback for updating the linear speed from a received Float32 message.
        
        Updates the internal linear speed state based on the incoming message.
        """
        self.linear_speed = msg.data
        self.get_logger().info(f'Linear speed set to: {self.linear_speed}')
    
    def angular_speed_callback(self, msg):
        """
        Callback to update the angular speed from a received Float32 message.
        """
        self.angular_speed = msg.data
        self.get_logger().info(f'Angular speed set to: {self.angular_speed}')
    
    def motion_active_callback(self, msg):
        """
        Callback to update the motion activation status based on received messages.
        
        Updates the internal state to reflect whether motion is active or inactive.
        """
        self.motion_active = msg.data
        status = "activated" if self.motion_active else "deactivated"
        self.get_logger().info(f'Motion {status}')
    
    def publish_velocity(self):
        """
        Publishes velocity commands to control the turtle's motion based on the current speed and activation status.
        
        If motion is inactive, sends a zero-velocity command to stop the turtle. Otherwise, publishes a `Twist` message with the current linear and angular speeds.
        """
        if not self.motion_active:
            # If motion is not active, publish zeros to stop the turtle
            twist_msg = Twist()
            self.velocity_publisher.publish(twist_msg)
            return
        
        # Create and publish the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear_speed)
        twist_msg.angular.z = float(self.angular_speed)
        
        self.velocity_publisher.publish(twist_msg)

def main(args=None):
    """
    Start the ROS 2 node for controlling turtle circular motion and manage its lifecycle.
    
    Initializes the ROS 2 client library, creates and spins the `CircularMotionBridge` node to process velocity commands, and ensures the turtle stops and resources are cleaned up on shutdown.
    """
    rclpy.init(args=args)
    node = CircularMotionBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Stop the turtle before shutting down
        stop_msg = Twist()
        node.velocity_publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 