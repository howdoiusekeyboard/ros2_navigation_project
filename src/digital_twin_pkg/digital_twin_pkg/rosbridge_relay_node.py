#!/usr/bin/env python3
"""
Rosbridge Relay Node

Connects to rosbridge WebSocket on the robot and relays topics to local ROS2.
This bypasses DDS discovery issues across subnets.

Subscribes via rosbridge:
    /odom -> publishes locally as /real/odom
    /scan -> publishes locally as /real/scan

Usage:
    ros2 run digital_twin_pkg rosbridge_relay_node --ros-args -p robot_ip:=10.30.96.171
"""

import json
import threading
import time
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist

try:
    import websocket
except ImportError:
    websocket = None


class RosbridgeRelayNode(Node):
    """Relays topics from remote rosbridge to local ROS2."""

    def __init__(self):
        super().__init__('rosbridge_relay_node')

        # Parameters
        self.declare_parameter('robot_ip', '10.30.96.171')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('reconnect_interval', 5.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.rosbridge_port = self.get_parameter('rosbridge_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        self.ws_url = f"ws://{self.robot_ip}:{self.rosbridge_port}"

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers (local)
        self.odom_pub = self.create_publisher(Odometry, '/real/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/real/scan', sensor_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/real/cmd_vel', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)

        # WebSocket connection
        self.ws: Optional[websocket.WebSocketApp] = None
        self.ws_connected = False
        self.ws_thread: Optional[threading.Thread] = None

        # Message ID counter
        self.msg_id = 0

        # Stats
        self.odom_count = 0
        self.scan_count = 0
        self.battery_count = 0

        # Start connection
        if websocket is None:
            self.get_logger().error("websocket-client not installed! Run: pip install websocket-client")
        else:
            self._start_websocket()

        # Status timer
        self.status_timer = self.create_timer(10.0, self._print_status)

        self.get_logger().info(f"RosbridgeRelayNode initialized")
        self.get_logger().info(f"Connecting to: {self.ws_url}")

    def _start_websocket(self):
        """Start WebSocket connection in background thread."""
        def on_open(ws):
            self.ws_connected = True
            self.get_logger().info(f"Connected to rosbridge at {self.ws_url}")
            # Subscribe to topics
            self._subscribe('/odom', 'nav_msgs/Odometry')
            self._subscribe('/scan', 'sensor_msgs/LaserScan')
            self._subscribe('/battery_state', 'sensor_msgs/BatteryState')

        def on_message(ws, message):
            try:
                data = json.loads(message)
                if data.get('op') == 'publish':
                    topic = data.get('topic')
                    msg_data = data.get('msg', {})
                    self._handle_message(topic, msg_data)
            except Exception as e:
                self.get_logger().warn(f"Error processing message: {e}")

        def on_error(ws, error):
            self.get_logger().warn(f"WebSocket error: {error}")

        def on_close(ws, close_status_code, close_msg):
            self.ws_connected = False
            self.get_logger().warn(f"WebSocket closed. Reconnecting in {self.reconnect_interval}s...")
            time.sleep(self.reconnect_interval)
            self._start_websocket()

        self.ws = websocket.WebSocketApp(
            self.ws_url,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close
        )

        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()

    def _subscribe(self, topic: str, msg_type: str):
        """Subscribe to a topic via rosbridge."""
        self.msg_id += 1
        subscribe_msg = {
            "op": "subscribe",
            "id": f"sub_{self.msg_id}",
            "topic": topic,
            "type": msg_type,
            "throttle_rate": 50,  # 20 Hz max
            "queue_length": 1
        }
        if self.ws and self.ws_connected:
            self.ws.send(json.dumps(subscribe_msg))
            self.get_logger().info(f"Subscribed to {topic}")

    def _handle_message(self, topic: str, msg_data: dict):
        """Handle incoming message from rosbridge."""
        if topic == '/odom':
            self._publish_odom(msg_data)
        elif topic == '/scan':
            self._publish_scan(msg_data)
        elif topic == '/battery_state':
            self._publish_battery(msg_data)

    def _publish_odom(self, data: dict):
        """Convert and publish odometry message."""
        try:
            msg = Odometry()
            
            # Header
            header = data.get('header', {})
            msg.header.stamp.sec = header.get('stamp', {}).get('sec', 0)
            msg.header.stamp.nanosec = header.get('stamp', {}).get('nanosec', 0)
            msg.header.frame_id = header.get('frame_id', 'odom')
            msg.child_frame_id = data.get('child_frame_id', 'base_footprint')

            # Pose
            pose = data.get('pose', {}).get('pose', {})
            pos = pose.get('position', {})
            msg.pose.pose.position.x = float(pos.get('x', 0))
            msg.pose.pose.position.y = float(pos.get('y', 0))
            msg.pose.pose.position.z = float(pos.get('z', 0))
            
            orient = pose.get('orientation', {})
            msg.pose.pose.orientation.x = float(orient.get('x', 0))
            msg.pose.pose.orientation.y = float(orient.get('y', 0))
            msg.pose.pose.orientation.z = float(orient.get('z', 0))
            msg.pose.pose.orientation.w = float(orient.get('w', 1))

            # Twist
            twist = data.get('twist', {}).get('twist', {})
            lin = twist.get('linear', {})
            msg.twist.twist.linear.x = float(lin.get('x', 0))
            msg.twist.twist.linear.y = float(lin.get('y', 0))
            msg.twist.twist.linear.z = float(lin.get('z', 0))
            
            ang = twist.get('angular', {})
            msg.twist.twist.angular.x = float(ang.get('x', 0))
            msg.twist.twist.angular.y = float(ang.get('y', 0))
            msg.twist.twist.angular.z = float(ang.get('z', 0))

            self.odom_pub.publish(msg)
            self.odom_count += 1

        except Exception as e:
            self.get_logger().warn(f"Error publishing odom: {e}")

    def _publish_scan(self, data: dict):
        """Convert and publish laser scan message."""
        try:
            msg = LaserScan()
            
            # Header
            header = data.get('header', {})
            msg.header.stamp.sec = header.get('stamp', {}).get('sec', 0) or 0
            msg.header.stamp.nanosec = header.get('stamp', {}).get('nanosec', 0) or 0
            msg.header.frame_id = header.get('frame_id', 'base_scan') or 'base_scan'

            # Scan parameters - handle None values
            msg.angle_min = float(data.get('angle_min') or 0)
            msg.angle_max = float(data.get('angle_max') or 0)
            msg.angle_increment = float(data.get('angle_increment') or 0)
            msg.time_increment = float(data.get('time_increment') or 0)
            msg.scan_time = float(data.get('scan_time') or 0)
            msg.range_min = float(data.get('range_min') or 0)
            msg.range_max = float(data.get('range_max') or 0)
            
            # Ranges and intensities - handle None/inf values
            ranges = data.get('ranges', [])
            if ranges:
                msg.ranges = [float(r) if r is not None else float('inf') for r in ranges]
            
            intensities = data.get('intensities', [])
            if intensities:
                msg.intensities = [float(i) if i is not None else 0.0 for i in intensities]

            self.scan_pub.publish(msg)
            self.scan_count += 1

        except Exception as e:
            self.get_logger().warn(f"Error publishing scan: {e}")

    def _publish_battery(self, data: dict):
        """Convert and publish battery state message."""
        try:
            msg = BatteryState()
            
            # Header
            header = data.get('header', {})
            msg.header.stamp.sec = header.get('stamp', {}).get('sec', 0) or 0
            msg.header.stamp.nanosec = header.get('stamp', {}).get('nanosec', 0) or 0
            msg.header.frame_id = header.get('frame_id', '') or ''

            msg.voltage = float(data.get('voltage') or 0)
            msg.current = float(data.get('current') or 0)
            msg.charge = float(data.get('charge') or 0)
            msg.capacity = float(data.get('capacity') or 0)
            msg.design_capacity = float(data.get('design_capacity') or 0)
            msg.percentage = float(data.get('percentage') or 0)
            msg.power_supply_status = int(data.get('power_supply_status') or 0)
            msg.power_supply_health = int(data.get('power_supply_health') or 0)
            msg.power_supply_technology = int(data.get('power_supply_technology') or 0)
            msg.present = bool(data.get('present', True))

            self.battery_pub.publish(msg)
            self.battery_count += 1

        except Exception as e:
            self.get_logger().warn(f"Error publishing battery: {e}")

    def _print_status(self):
        """Print status every 10 seconds."""
        status = "CONNECTED" if self.ws_connected else "DISCONNECTED"
        self.get_logger().info(
            f"Status: {status} | Odom: {self.odom_count} | Scan: {self.scan_count} | Battery: {self.battery_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RosbridgeRelayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ws:
            node.ws.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
