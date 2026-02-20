#!/usr/bin/env python3
"""
XAI Bridge Node

Connects to robot's rosbridge WebSocket and relays Nav2 topics to local ROS2.
This enables the XAI Navigator to run on WSL while monitoring the robot's navigation.

Relays from robot (via rosbridge):
    /plan -> /plan (local)
    /local_plan -> /local_plan (local)
    /goal_pose -> /goal_pose (local)
    /local_costmap/costmap -> /local_costmap/costmap (local)
    /global_costmap/costmap -> /global_costmap/costmap (local)
    /navigate_to_pose/feedback -> (processed for XAI)
    /navigate_to_pose/status -> (processed for XAI)

Also relays sensor data:
    /odom -> /real/odom (local)
    /scan -> /real/scan (local)
    /battery_state -> /battery_state (local)

Usage:
    ros2 run digital_twin_pkg xai_bridge_node --ros-args -p robot_ip:=10.30.96.171
"""

import json
import math
import threading
import time
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String, Header
from builtin_interfaces.msg import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

try:
    import websocket
except ImportError:
    websocket = None


class XAIBridgeNode(Node):
    """Bridges Nav2 topics from robot's rosbridge to local ROS2 for XAI processing."""

    def __init__(self):
        super().__init__('xai_bridge_node')

        # Parameters
        self.declare_parameter('robot_ip', '10.30.96.171')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('reconnect_interval', 5.0)

        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.rosbridge_port = int(self.get_parameter('rosbridge_port').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)

        self.ws_url = f"ws://{self.robot_ip}:{self.rosbridge_port}"

        if websocket is None:
            self.get_logger().error("websocket-client not installed! Run: pip install websocket-client")
            return

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        transient_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Increased depth to match RViz2 default
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # QoS profile specifically for RViz2 compatibility (matches RViz2's /initialpose)
        rviz_initialpose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers - Navigation topics
        self.plan_pub = self.create_publisher(Path, '/plan', reliable_qos)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', reliable_qos)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', reliable_qos)
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', reliable_qos)
        self.global_costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', reliable_qos)

        # Publishers - Navigation status (for XAI to monitor)
        self.nav_feedback_pub = self.create_publisher(String, '/navigate_to_pose/feedback_json', reliable_qos)
        self.nav_status_pub = self.create_publisher(String, '/navigate_to_pose/status_json', reliable_qos)

        # Publishers - Sensor data (for dashboard)
        self.odom_pub = self.create_publisher(Odometry, '/real/odom', reliable_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/real/scan', sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', reliable_qos)

        # Publisher - Initial pose (for AMCL)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', transient_qos
        )

        # Publishers - TF for RViz2 visualization
        self.tf_pub = self.create_publisher(TFMessage, '/tf', reliable_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', transient_qos)
        
        # Static transform broadcaster for map frame (workaround)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher - Map for RViz2
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', transient_qos)

        # Subscribers - Forward goals/poses FROM WSL TO robot
        # initialpose needs TRANSIENT_LOCAL to match RViz2's QoS
        # goal_pose uses reliable QoS from RViz2
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._forward_goal_to_robot, reliable_qos
        )
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._forward_initialpose_to_robot, rviz_initialpose_qos
        )

        # WebSocket state
        self.ws: Optional[websocket.WebSocketApp] = None
        self.ws_connected = False
        self.ws_thread: Optional[threading.Thread] = None
        self.running = True

        # Statistics
        self.stats = {
            'odom': 0,
            'scan': 0,
            'battery': 0,
            'plan': 0,
            'local_plan': 0,
            'goal': 0,
            'costmap_local': 0,
            'costmap_global': 0,
            'nav_feedback': 0,
            'nav_status': 0,
            'tf': 0,
            'tf_static': 0,
            'map': 0,
        }

        # Start WebSocket connection
        self._start_websocket()

        # Status timer
        self.status_timer = self.create_timer(10.0, self._print_status)
        
        # Map request timer (latched topics don't work well with rosbridge subscriptions)
        self.map_request_timer = self.create_timer(5.0, self._request_map_via_service)
        self.map_received = False
        
        # Test TF publisher (publish a dummy transform to verify publisher works)
        self.test_tf_timer = self.create_timer(1.0, self._publish_test_tf)
        self.tf_test_count = 0

        self.get_logger().info(f"XAI Bridge Node initialized")
        self.get_logger().info(f"Connecting to robot rosbridge at {self.ws_url}")

    def _start_websocket(self):
        """Start WebSocket connection in background thread."""
        def run_ws():
            while self.running:
                try:
                    self.ws = websocket.WebSocketApp(
                        self.ws_url,
                        on_open=self._on_open,
                        on_message=self._on_message,
                        on_error=self._on_error,
                        on_close=self._on_close
                    )
                    self.ws.run_forever()
                except Exception as e:
                    self.get_logger().error(f"WebSocket error: {e}")
                
                if self.running:
                    self.get_logger().info(f"Reconnecting in {self.reconnect_interval}s...")
                    time.sleep(self.reconnect_interval)

        self.ws_thread = threading.Thread(target=run_ws, daemon=True)
        self.ws_thread.start()

    def _on_open(self, ws):
        """Called when WebSocket connects."""
        self.ws_connected = True
        self.get_logger().info(f"Connected to rosbridge at {self.ws_url}")

        # Subscribe to all relevant topics
        topics = [
            # Sensor data
            ('/odom', 'nav_msgs/Odometry'),
            ('/scan', 'sensor_msgs/LaserScan'),
            ('/battery_state', 'sensor_msgs/BatteryState'),
            
            # Navigation planning
            ('/plan', 'nav_msgs/Path'),
            ('/local_plan', 'nav_msgs/Path'),
            ('/goal_pose', 'geometry_msgs/PoseStamped'),
            
            # Costmaps
            ('/local_costmap/costmap', 'nav_msgs/OccupancyGrid'),
            ('/global_costmap/costmap', 'nav_msgs/OccupancyGrid'),
            
            # Navigation action feedback
            ('/navigate_to_pose/_action/feedback', 'nav2_msgs/action/NavigateToPose_FeedbackMessage'),
            ('/navigate_to_pose/_action/status', 'action_msgs/msg/GoalStatusArray'),
            
            # TF for RViz2 visualization
            ('/tf', 'tf2_msgs/TFMessage'),
            ('/tf_static', 'tf2_msgs/TFMessage'),
            
            # Map for RViz2
            ('/map', 'nav_msgs/OccupancyGrid'),
        ]

        for topic, msg_type in topics:
            self._subscribe(topic, msg_type)

    def _subscribe(self, topic: str, msg_type: str):
        """Subscribe to a topic via rosbridge."""
        subscribe_msg = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type
        }
        try:
            self.ws.send(json.dumps(subscribe_msg))
            self.get_logger().debug(f"Subscribed to {topic}")
        except Exception as e:
            self.get_logger().error(f"Subscribe error for {topic}: {e}")

    def _request_map_via_service(self):
        """Request map via service call (workaround for latched topic issue)."""
        if not self.ws_connected or self.map_received:
            return
        
        # Call /map_server/map service to get the map
        service_call = {
            "op": "call_service",
            "service": "/map_server/map",
            "type": "nav_msgs/srv/GetMap",
            "args": {}
        }
        
        try:
            self.ws.send(json.dumps(service_call))
            self.get_logger().info("Requesting map via /map_server/map service...")
        except Exception as e:
            self.get_logger().warn(f"Failed to request map: {e}")

    def _on_message(self, ws, message: str):
        """Handle incoming WebSocket message."""
        try:
            data = json.loads(message)
            op = data.get('op', '')
            
            if op == 'publish':
                topic = data.get('topic', '')
                msg_data = data.get('msg', {})
                self._handle_message(topic, msg_data)
            elif op == 'service_response':
                # Handle service call responses (e.g., map service)
                service = data.get('service', '')
                if service == '/map_server/map':
                    self._handle_map_service_response(data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().warn(f"Message handling error: {e}")

    def _on_error(self, ws, error):
        """Handle WebSocket error."""
        self.get_logger().error(f"WebSocket error: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        """Handle WebSocket close."""
        self.ws_connected = False
        self.map_received = False  # Reset so we request map again on reconnect
        self.get_logger().warn(f"WebSocket closed: {close_status_code} - {close_msg}")

    def _handle_map_service_response(self, data: dict):
        """Handle response from /map_server/map service."""
        try:
            # Debug: log the full response structure
            self.get_logger().info(f"Map service response keys: {data.keys()}")
            
            # Check for success/result field
            if data.get('result') is False or data.get('success') is False:
                self.get_logger().warn(f"Map service call failed: {data}")
                return
            
            # Try different response formats
            values = data.get('values', data)
            map_data = values.get('map', values.get('response', {}).get('map', {}))
            
            if not map_data and 'map' not in str(data):
                # Maybe the whole response IS the map
                if 'header' in data or 'info' in data:
                    map_data = data
            
            if map_data:
                self._publish_map(map_data)
                self.map_received = True
                self.get_logger().info("Successfully received map via service call")
            else:
                self.get_logger().warn(f"Map service returned unexpected format: {list(data.keys())}")
        except Exception as e:
            self.get_logger().error(f"Error handling map service response: {e}")

    def _handle_message(self, topic: str, msg_data: dict):
        """Route incoming message to appropriate handler."""
        handlers = {
            '/odom': self._publish_odom,
            '/scan': self._publish_scan,
            '/battery_state': self._publish_battery,
            '/plan': self._publish_plan,
            '/local_plan': self._publish_local_plan,
            '/goal_pose': self._publish_goal_pose,
            '/local_costmap/costmap': self._publish_local_costmap,
            '/global_costmap/costmap': self._publish_global_costmap,
            '/navigate_to_pose/_action/feedback': self._publish_nav_feedback,
            '/navigate_to_pose/_action/status': self._publish_nav_status,
            '/tf': self._publish_tf,
            '/tf_static': self._publish_tf_static,
            '/map': self._publish_map,
        }

        handler = handlers.get(topic)
        if handler:
            handler(msg_data)

    def _parse_header(self, data: dict) -> Header:
        """Parse header from message data."""
        header = Header()
        header_data = data.get('header', {})
        stamp = header_data.get('stamp', {})
        header.stamp.sec = int(stamp.get('sec', 0) or 0)
        header.stamp.nanosec = int(stamp.get('nanosec', 0) or 0)
        header.frame_id = str(header_data.get('frame_id', '') or '')
        return header

    def _publish_odom(self, data: dict):
        """Publish odometry message."""
        try:
            msg = Odometry()
            msg.header = self._parse_header(data)
            
            pose = data.get('pose', {}).get('pose', {})
            pos = pose.get('position', {})
            ori = pose.get('orientation', {})
            
            msg.pose.pose.position.x = float(pos.get('x', 0) or 0)
            msg.pose.pose.position.y = float(pos.get('y', 0) or 0)
            msg.pose.pose.position.z = float(pos.get('z', 0) or 0)
            msg.pose.pose.orientation.x = float(ori.get('x', 0) or 0)
            msg.pose.pose.orientation.y = float(ori.get('y', 0) or 0)
            msg.pose.pose.orientation.z = float(ori.get('z', 0) or 0)
            msg.pose.pose.orientation.w = float(ori.get('w', 1) or 1)

            twist = data.get('twist', {}).get('twist', {})
            lin = twist.get('linear', {})
            ang = twist.get('angular', {})
            
            msg.twist.twist.linear.x = float(lin.get('x', 0) or 0)
            msg.twist.twist.linear.y = float(lin.get('y', 0) or 0)
            msg.twist.twist.linear.z = float(lin.get('z', 0) or 0)
            msg.twist.twist.angular.x = float(ang.get('x', 0) or 0)
            msg.twist.twist.angular.y = float(ang.get('y', 0) or 0)
            msg.twist.twist.angular.z = float(ang.get('z', 0) or 0)

            self.odom_pub.publish(msg)
            self.stats['odom'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing odom: {e}")

    def _publish_scan(self, data: dict):
        """Publish laser scan message."""
        try:
            msg = LaserScan()
            msg.header = self._parse_header(data)
            
            msg.angle_min = float(data.get('angle_min', 0) or 0)
            msg.angle_max = float(data.get('angle_max', 0) or 0)
            msg.angle_increment = float(data.get('angle_increment', 0) or 0)
            msg.time_increment = float(data.get('time_increment', 0) or 0)
            msg.scan_time = float(data.get('scan_time', 0) or 0)
            msg.range_min = float(data.get('range_min', 0) or 0)
            msg.range_max = float(data.get('range_max', 0) or 0)

            ranges = data.get('ranges', [])
            msg.ranges = [float(r) if r is not None and not math.isinf(float(r)) else float('inf') for r in ranges]
            
            intensities = data.get('intensities', [])
            if intensities:
                msg.intensities = [float(i) if i is not None else 0.0 for i in intensities]

            self.scan_pub.publish(msg)
            self.stats['scan'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing scan: {e}")

    def _publish_battery(self, data: dict):
        """Publish battery state message."""
        try:
            msg = BatteryState()
            msg.header = self._parse_header(data)
            msg.voltage = float(data.get('voltage', 0) or 0)
            msg.current = float(data.get('current', 0) or 0)
            msg.percentage = float(data.get('percentage', 0) or 0)
            msg.present = bool(data.get('present', True))
            
            self.battery_pub.publish(msg)
            self.stats['battery'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing battery: {e}")

    def _publish_plan(self, data: dict):
        """Publish global path plan."""
        try:
            msg = self._parse_path(data)
            self.plan_pub.publish(msg)
            self.stats['plan'] += 1
            self.get_logger().info(f"Received global plan with {len(msg.poses)} poses")
        except Exception as e:
            self.get_logger().warn(f"Error publishing plan: {e}")

    def _publish_local_plan(self, data: dict):
        """Publish local path plan."""
        try:
            msg = self._parse_path(data)
            self.local_plan_pub.publish(msg)
            self.stats['local_plan'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing local_plan: {e}")

    def _parse_path(self, data: dict) -> Path:
        """Parse Path message from data."""
        msg = Path()
        msg.header = self._parse_header(data)
        
        poses = data.get('poses', [])
        for pose_data in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = self._parse_header(pose_data)
            
            pose = pose_data.get('pose', {})
            pos = pose.get('position', {})
            ori = pose.get('orientation', {})
            
            pose_stamped.pose.position.x = float(pos.get('x', 0) or 0)
            pose_stamped.pose.position.y = float(pos.get('y', 0) or 0)
            pose_stamped.pose.position.z = float(pos.get('z', 0) or 0)
            pose_stamped.pose.orientation.x = float(ori.get('x', 0) or 0)
            pose_stamped.pose.orientation.y = float(ori.get('y', 0) or 0)
            pose_stamped.pose.orientation.z = float(ori.get('z', 0) or 0)
            pose_stamped.pose.orientation.w = float(ori.get('w', 1) or 1)
            
            msg.poses.append(pose_stamped)
        
        return msg

    def _publish_goal_pose(self, data: dict):
        """Publish goal pose."""
        try:
            msg = PoseStamped()
            msg.header = self._parse_header(data)
            
            pose = data.get('pose', {})
            pos = pose.get('position', {})
            ori = pose.get('orientation', {})
            
            msg.pose.position.x = float(pos.get('x', 0) or 0)
            msg.pose.position.y = float(pos.get('y', 0) or 0)
            msg.pose.position.z = float(pos.get('z', 0) or 0)
            msg.pose.orientation.x = float(ori.get('x', 0) or 0)
            msg.pose.orientation.y = float(ori.get('y', 0) or 0)
            msg.pose.orientation.z = float(ori.get('z', 0) or 0)
            msg.pose.orientation.w = float(ori.get('w', 1) or 1)
            
            self.goal_pose_pub.publish(msg)
            self.stats['goal'] += 1
            self.get_logger().info(f"Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        except Exception as e:
            self.get_logger().warn(f"Error publishing goal_pose: {e}")

    def _publish_local_costmap(self, data: dict):
        """Publish local costmap."""
        try:
            msg = self._parse_occupancy_grid(data)
            self.local_costmap_pub.publish(msg)
            self.stats['costmap_local'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing local costmap: {e}")

    def _publish_global_costmap(self, data: dict):
        """Publish global costmap."""
        try:
            msg = self._parse_occupancy_grid(data)
            self.global_costmap_pub.publish(msg)
            self.stats['costmap_global'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing global costmap: {e}")

    def _parse_occupancy_grid(self, data: dict) -> OccupancyGrid:
        """Parse OccupancyGrid message from data."""
        msg = OccupancyGrid()
        msg.header = self._parse_header(data)
        
        info = data.get('info', {})
        msg.info.resolution = float(info.get('resolution', 0.05) or 0.05)
        msg.info.width = int(info.get('width', 0) or 0)
        msg.info.height = int(info.get('height', 0) or 0)
        
        origin = info.get('origin', {})
        pos = origin.get('position', {})
        ori = origin.get('orientation', {})
        
        msg.info.origin.position.x = float(pos.get('x', 0) or 0)
        msg.info.origin.position.y = float(pos.get('y', 0) or 0)
        msg.info.origin.position.z = float(pos.get('z', 0) or 0)
        msg.info.origin.orientation.x = float(ori.get('x', 0) or 0)
        msg.info.origin.orientation.y = float(ori.get('y', 0) or 0)
        msg.info.origin.orientation.z = float(ori.get('z', 0) or 0)
        msg.info.origin.orientation.w = float(ori.get('w', 1) or 1)
        
        grid_data = data.get('data', [])
        msg.data = [int(d) if d is not None else -1 for d in grid_data]
        
        return msg

    def _publish_nav_feedback(self, data: dict):
        """Publish navigation feedback as JSON for XAI processing."""
        try:
            msg = String()
            msg.data = json.dumps(data)
            self.nav_feedback_pub.publish(msg)
            self.stats['nav_feedback'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing nav feedback: {e}")

    def _publish_nav_status(self, data: dict):
        """Publish navigation status as JSON for XAI processing."""
        try:
            msg = String()
            msg.data = json.dumps(data)
            self.nav_status_pub.publish(msg)
            self.stats['nav_status'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing nav status: {e}")

    def _publish_tf(self, data: dict):
        """Publish TF transforms for RViz2."""
        try:
            msg = TFMessage()
            transforms = data.get('transforms', [])
            
            if not transforms:
                # Try alternative format - maybe data IS the transform
                if 'header' in data or 'child_frame_id' in data:
                    transforms = [data]
                else:
                    self.get_logger().debug(f"TF message has no transforms: {list(data.keys())}")
                    return
            
            for tf_data in transforms:
                tf = TransformStamped()
                
                # Header
                header = tf_data.get('header', {})
                if not header and 'frame_id' in tf_data:
                    # Alternative format
                    header = {'frame_id': tf_data.get('frame_id', '')}
                    stamp = tf_data.get('stamp', {})
                else:
                    stamp = header.get('stamp', {})
                
                tf.header.stamp.sec = int(stamp.get('sec', 0) or 0)
                tf.header.stamp.nanosec = int(stamp.get('nanosec', 0) or 0)
                tf.header.frame_id = str(header.get('frame_id', '') or '')
                
                tf.child_frame_id = str(tf_data.get('child_frame_id', '') or '')
                
                # Transform
                transform = tf_data.get('transform', {})
                trans = transform.get('translation', {})
                rot = transform.get('rotation', {})
                
                tf.transform.translation.x = float(trans.get('x', 0) or 0)
                tf.transform.translation.y = float(trans.get('y', 0) or 0)
                tf.transform.translation.z = float(trans.get('z', 0) or 0)
                tf.transform.rotation.x = float(rot.get('x', 0) or 0)
                tf.transform.rotation.y = float(rot.get('y', 0) or 0)
                tf.transform.rotation.z = float(rot.get('z', 0) or 0)
                tf.transform.rotation.w = float(rot.get('w', 1) or 1)
                
                msg.transforms.append(tf)
            
            if msg.transforms:
                self.tf_pub.publish(msg)
                self.stats['tf'] += 1
                if self.stats['tf'] % 100 == 0:  # Log every 100th message
                    self.get_logger().debug(f"Published TF with {len(msg.transforms)} transforms")
        except Exception as e:
            self.get_logger().warn(f"Error publishing tf: {e}", exc_info=True)

    def _publish_tf_static(self, data: dict):
        """Publish static TF transforms for RViz2."""
        try:
            msg = TFMessage()
            transforms = data.get('transforms', [])
            
            for tf_data in transforms:
                tf = TransformStamped()
                
                header = tf_data.get('header', {})
                stamp = header.get('stamp', {})
                tf.header.stamp.sec = int(stamp.get('sec', 0) or 0)
                tf.header.stamp.nanosec = int(stamp.get('nanosec', 0) or 0)
                tf.header.frame_id = str(header.get('frame_id', '') or '')
                
                tf.child_frame_id = str(tf_data.get('child_frame_id', '') or '')
                
                transform = tf_data.get('transform', {})
                trans = transform.get('translation', {})
                rot = transform.get('rotation', {})
                
                tf.transform.translation.x = float(trans.get('x', 0) or 0)
                tf.transform.translation.y = float(trans.get('y', 0) or 0)
                tf.transform.translation.z = float(trans.get('z', 0) or 0)
                tf.transform.rotation.x = float(rot.get('x', 0) or 0)
                tf.transform.rotation.y = float(rot.get('y', 0) or 0)
                tf.transform.rotation.z = float(rot.get('z', 0) or 0)
                tf.transform.rotation.w = float(rot.get('w', 1) or 1)
                
                msg.transforms.append(tf)
            
            self.tf_static_pub.publish(msg)
            self.stats['tf_static'] += 1
        except Exception as e:
            self.get_logger().warn(f"Error publishing tf_static: {e}")

    def _publish_test_tf(self):
        """Publish a test TF transform to verify the publisher works."""
        try:
            msg = TFMessage()
            tf = TransformStamped()
            
            now = self.get_clock().now()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = "map"
            tf.child_frame_id = "odom"
            
            # Identity transform (will be overwritten by real TF when it arrives)
            tf.transform.translation.x = 0.0
            tf.transform.translation.y = 0.0
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
            tf.transform.rotation.w = 1.0
            
            msg.transforms.append(tf)
            self.tf_pub.publish(msg)
            
            self.tf_test_count += 1
            if self.tf_test_count == 1:
                self.get_logger().info("Published test TF transform (map->odom)")
        except Exception as e:
            self.get_logger().warn(f"Error publishing test TF: {e}")

    def _publish_map(self, data: dict):
        """Publish map for RViz2."""
        try:
            msg = self._parse_occupancy_grid(data)
            self.map_pub.publish(msg)
            self.stats['map'] += 1
            self.get_logger().info(f"Received and published map (total: {self.stats['map']})")
        except Exception as e:
            self.get_logger().warn(f"Error publishing map: {e}")

    def _print_status(self):
        """Print status every 10 seconds."""
        status = "CONNECTED" if self.ws_connected else "DISCONNECTED"
        self.get_logger().info(
            f"Status: {status} | "
            f"Odom: {self.stats['odom']} | "
            f"Scan: {self.stats['scan']} | "
            f"TF: {self.stats['tf']} | "
            f"Map: {self.stats['map']} | "
            f"Plan: {self.stats['plan']} | "
            f"Costmap: L{self.stats['costmap_local']}/G{self.stats['costmap_global']}"
        )

    def _forward_goal_to_robot(self, msg: PoseStamped):
        """Forward goal_pose from RViz2 to robot via rosbridge."""
        self.get_logger().info(f"Received goal_pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        if not self.ws_connected:
            self.get_logger().warn("Cannot forward goal - not connected to robot")
            return
        
        goal_msg = {
            "op": "publish",
            "topic": "/goal_pose",
            "msg": {
                "header": {
                    "frame_id": msg.header.frame_id or "map",
                    "stamp": {
                        "sec": msg.header.stamp.sec,
                        "nanosec": msg.header.stamp.nanosec
                    }
                },
                "pose": {
                    "position": {
                        "x": msg.pose.position.x,
                        "y": msg.pose.position.y,
                        "z": msg.pose.position.z
                    },
                    "orientation": {
                        "x": msg.pose.orientation.x,
                        "y": msg.pose.orientation.y,
                        "z": msg.pose.orientation.z,
                        "w": msg.pose.orientation.w
                    }
                }
            }
        }
        
        try:
            self.ws.send(json.dumps(goal_msg))
            self.get_logger().info(
                f"Forwarded goal to robot: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to forward goal: {e}")

    def _forward_initialpose_to_robot(self, msg: PoseWithCovarianceStamped):
        """Forward initialpose from RViz2 to robot via rosbridge."""
        self.get_logger().info(f"Received initialpose: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
        if not self.ws_connected:
            self.get_logger().warn("Cannot forward initialpose - not connected to robot")
            return
        
        pose_msg = {
            "op": "publish",
            "topic": "/initialpose",
            "msg": {
                "header": {
                    "frame_id": msg.header.frame_id or "map",
                    "stamp": {
                        "sec": msg.header.stamp.sec,
                        "nanosec": msg.header.stamp.nanosec
                    }
                },
                "pose": {
                    "pose": {
                        "position": {
                            "x": msg.pose.pose.position.x,
                            "y": msg.pose.pose.position.y,
                            "z": msg.pose.pose.position.z
                        },
                        "orientation": {
                            "x": msg.pose.pose.orientation.x,
                            "y": msg.pose.pose.orientation.y,
                            "z": msg.pose.pose.orientation.z,
                            "w": msg.pose.pose.orientation.w
                        }
                    },
                    "covariance": list(msg.pose.covariance)
                }
            }
        }
        
        try:
            self.ws.send(json.dumps(pose_msg))
            self.get_logger().info(
                f"Forwarded initialpose to robot: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to forward initialpose: {e}")

    def send_goal(self, x: float, y: float, theta: float = 0.0):
        """Send navigation goal to robot via rosbridge."""
        if not self.ws_connected:
            self.get_logger().error("Not connected to rosbridge")
            return False

        # Quaternion from yaw
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        goal_msg = {
            "op": "publish",
            "topic": "/goal_pose",
            "msg": {
                "header": {
                    "frame_id": "map",
                    "stamp": {"sec": 0, "nanosec": 0}
                },
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}
                }
            }
        }

        try:
            self.ws.send(json.dumps(goal_msg))
            self.get_logger().info(f"Sent goal: ({x:.2f}, {y:.2f}, θ={theta:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            return False

    def set_initial_pose(self, x: float, y: float, theta: float = 0.0):
        """Set initial pose for AMCL localization."""
        if not self.ws_connected:
            self.get_logger().error("Not connected to rosbridge")
            return False

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        pose_msg = {
            "op": "publish",
            "topic": "/initialpose",
            "msg": {
                "header": {
                    "frame_id": "map",
                    "stamp": {"sec": 0, "nanosec": 0}
                },
                "pose": {
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}
                    },
                    "covariance": [0.25, 0, 0, 0, 0, 0,
                                   0, 0.25, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0.0685]
                }
            }
        }

        try:
            self.ws.send(json.dumps(pose_msg))
            self.get_logger().info(f"Set initial pose: ({x:.2f}, {y:.2f}, θ={theta:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set initial pose: {e}")
            return False

    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        if self.ws:
            self.ws.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XAIBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
