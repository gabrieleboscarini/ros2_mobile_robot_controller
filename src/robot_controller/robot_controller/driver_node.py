from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_interfaces.msg import ObstacleInfo
from robot_interfaces.srv import GetAverageVelocity, SetVelocity

HISTORY_SIZE = 5


class DriverNode(Node):

    def __init__(self):
        super().__init__('driver_node')
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.reversed = False
        self.velocity_history = deque(maxlen=HISTORY_SIZE)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.srv = self.create_service(
            SetVelocity, 'set_velocity', self.set_velocity_callback
        )
        self.average_srv = self.create_service(
            GetAverageVelocity, 'get_average_velocity', self.get_average_velocity_callback
        )
        self.obstacle_sub = self.create_subscription(
            ObstacleInfo, 'obstacle_info', self.obstacle_info_callback, 10
        )

        self.get_logger().info(
            'Driver node started. Set velocity with: '
            'ros2 service call /set_velocity robot_interfaces/srv/SetVelocity '
            '"{linear: , angular: }"'
        )

    def publish_velocity(self):  #publisher callback
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher_.publish(msg)

    def set_velocity_callback(self, request, response): #service callback
        self.linear_vel = request.linear
        self.angular_vel = request.angular
        self.reversed = False
        self.velocity_history.append((self.linear_vel, self.angular_vel))
        response.success = True
        response.message = (
            f'Velocity set to linear={self.linear_vel}, angular={self.angular_vel}'
        )
        self.get_logger().info(response.message)
        return response

    def get_average_velocity_callback(self, request, response):
        if self.velocity_history:
            linear_values, angular_values = zip(*self.velocity_history)
            response.average_linear = sum(linear_values) / len(linear_values)
            response.average_angular = sum(angular_values) / len(angular_values)
        else:
            response.average_linear = 0.0
            response.average_angular = 0.0
        return response

    def obstacle_info_callback(self, msg):
        if msg.distance < msg.threshold and not self.reversed:
            self.linear_vel = -self.linear_vel
            self.angular_vel = -self.angular_vel
            self.reversed = True
            self.get_logger().warn(
                f'Obstacle too close ({msg.distance:.2f} m < {msg.threshold:.2f} m) '
                f'on the {msg.direction}, reversing to '
                f'linear={self.linear_vel}, angular={self.angular_vel}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()

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
