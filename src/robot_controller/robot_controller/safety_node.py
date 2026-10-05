import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from robot_interfaces.msg import ObstacleInfo
from robot_interfaces.srv import SetThreshold

FRONT_HALF_WIDTH = math.pi / 4  # obstacles within +/-45 deg of the x axis count as "front"


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')
        self.threshold = 0.5
        self.min_distance = float('inf')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.obstacle_pub = self.create_publisher(ObstacleInfo, 'obstacle_info', 10)
        self.threshold_srv = self.create_service(
            SetThreshold, 'set_threshold', self.set_threshold_callback
        )

        self.get_logger().info('Safety node started, listening on /scan.')

    def scan_callback(self, msg):
        valid = [(i, r) for i, r in enumerate(msg.ranges) if math.isfinite(r)]

        if valid:
            min_index, min_range = min(valid, key=lambda pair: pair[1])
            self.min_distance = min_range
            angle = msg.angle_min + min_index * msg.angle_increment
        else:
            # Nothing within range_max on any beam: report max range, no clear direction.
            self.min_distance = msg.range_max
            angle = 0.0

        obstacle_msg = ObstacleInfo()
        obstacle_msg.distance = self.min_distance
        obstacle_msg.direction = self.get_direction(angle)
        obstacle_msg.threshold = self.threshold
        self.obstacle_pub.publish(obstacle_msg)

    def set_threshold_callback(self, request, response):
        self.threshold = request.threshold
        response.success = True
        response.message = f'Threshold set to {self.threshold}'
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def get_direction(angle):
        angle = math.atan2(math.sin(angle), math.cos(angle))  # normalize to (-pi, pi]
        if -FRONT_HALF_WIDTH <= angle <= FRONT_HALF_WIDTH:
            return 'front'
        elif angle > FRONT_HALF_WIDTH:
            return 'left'
        else:
            return 'right'


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
