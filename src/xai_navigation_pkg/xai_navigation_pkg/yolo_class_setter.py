#!/usr/bin/env python3
"""
YOLO Class Setter Node for Tesla-Style Obstacle Classification

This node calls the yolo_ros set_classes service to configure YOLO-World
for Tesla-style obstacle detection with priority-based classification.

Classes are organized by priority tier:
- Tier 1 (Human, Priority 10.0): person, pedestrian, child, wheelchair user
- Tier 2 (Vehicle, Priority 5.0): wheelchair, cart, bicycle, scooter, stroller
- Tier 3 (Furniture, Priority 2.0): chair, table, couch, desk, box, cabinet
- Tier 4 (Wall, Priority 1.0): wall, pillar, door

This node runs once on startup, calling the service and then remaining
available for potential runtime reconfiguration.
"""

import rclpy
from rclpy.node import Node
from yolo_msgs.srv import SetClasses


class YoloClassSetter(Node):
    """
    ROS2 Node that configures YOLO-World classes for obstacle classification.

    Waits for the yolo_ros set_classes service to become available,
    then configures the model with Tesla-style priority classes.
    """

    # Tesla-style priority class definitions
    PRIORITY_CLASSES = {
        'human': [
            'person',
            'pedestrian',
            'child',
            'wheelchair user',
            'man',
            'woman',
        ],
        'vehicle': [
            'wheelchair',
            'cart',
            'shopping cart',
            'bicycle',
            'scooter',
            'stroller',
            'robot',
        ],
        'furniture': [
            'chair',
            'office chair',
            'table',
            'desk',
            'couch',
            'sofa',
            'box',
            'cabinet',
            'shelf',
            'bench',
        ],
        'wall': [
            'wall',
            'pillar',
            'door',
            'column',
        ],
    }

    # Priority weights matching obstacle_classifier.py
    PRIORITY_WEIGHTS = {
        'human': 10.0,
        'vehicle': 5.0,
        'dynamic': 3.0,
        'furniture': 2.0,
        'wall': 1.0,
        'unknown': 1.5,
    }

    def __init__(self):
        super().__init__('yolo_class_setter')

        # Build flat list of all classes for YOLO-World
        self.classes = []
        self.class_to_priority_type = {}

        for priority_type, class_list in self.PRIORITY_CLASSES.items():
            for class_name in class_list:
                self.classes.append(class_name)
                self.class_to_priority_type[class_name] = priority_type

        self.get_logger().info(
            f'Prepared {len(self.classes)} classes across '
            f'{len(self.PRIORITY_CLASSES)} priority tiers'
        )

        # Service client for yolo_ros
        self.client = self.create_client(SetClasses, '/yolo/set_classes')

        # Timer to retry service call until successful
        self.timer = self.create_timer(2.0, self.try_set_classes)
        self.classes_set = False
        self.in_flight = False
        self.retry_count = 0
        self.max_retries = 30  # Give up after 60 seconds

        # Publisher for class mapping (useful for other nodes)
        self.declare_parameter('publish_mapping', True)

    def try_set_classes(self):
        """Attempt to set YOLO-World classes via service call."""
        if self.classes_set or self.in_flight:
            return

        self.retry_count += 1

        if self.retry_count > self.max_retries:
            self.get_logger().error(
                f'Failed to set YOLO classes after {self.max_retries} attempts. '
                'Is yolo_ros running?'
            )
            self.timer.cancel()
            return

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Waiting for /yolo/set_classes service... '
                f'(attempt {self.retry_count}/{self.max_retries})'
            )
            return

        # Create and send request
        request = SetClasses.Request()
        request.classes = self.classes

        self.in_flight = True
        self.get_logger().info(f'Calling set_classes with {len(self.classes)} classes...')
        future = self.client.call_async(request)
        future.add_done_callback(self.classes_callback)

    def classes_callback(self, future):
        """Handle service response."""
        self.in_flight = False
        try:
            future.result()  # SetClasses has empty response

            self.classes_set = True
            self.timer.cancel()

            self.get_logger().info(
                '\n' + '=' * 60 + '\n'
                '  YOLO-World Classes Configured Successfully!\n'
                '=' * 60 + '\n'
                f'  Total classes: {len(self.classes)}\n'
                f'  Human tier ({self.PRIORITY_WEIGHTS["human"]}): '
                f'{len(self.PRIORITY_CLASSES["human"])} classes\n'
                f'  Vehicle tier ({self.PRIORITY_WEIGHTS["vehicle"]}): '
                f'{len(self.PRIORITY_CLASSES["vehicle"])} classes\n'
                f'  Furniture tier ({self.PRIORITY_WEIGHTS["furniture"]}): '
                f'{len(self.PRIORITY_CLASSES["furniture"])} classes\n'
                f'  Wall tier ({self.PRIORITY_WEIGHTS["wall"]}): '
                f'{len(self.PRIORITY_CLASSES["wall"])} classes\n'
                '=' * 60
            )

            # Log the class-to-priority mapping for debugging
            self.get_logger().debug(
                f'Class mapping: {self.class_to_priority_type}'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to set classes: {e}')
            # Will retry on next timer tick

    def get_priority_type(self, class_name: str) -> str:
        """
        Get the priority type for a detected class name.

        Args:
            class_name: The class name from YOLO detection

        Returns:
            Priority type string ('human', 'vehicle', 'furniture', 'wall', 'unknown')
        """
        return self.class_to_priority_type.get(class_name.lower(), 'unknown')

    def get_priority_weight(self, class_name: str) -> float:
        """
        Get the priority weight for a detected class name.

        Args:
            class_name: The class name from YOLO detection

        Returns:
            Priority weight (10.0 for human, 5.0 for vehicle, etc.)
        """
        priority_type = self.get_priority_type(class_name)
        return self.PRIORITY_WEIGHTS.get(priority_type, 1.5)


def main(args=None):
    rclpy.init(args=args)

    node = YoloClassSetter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
