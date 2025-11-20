import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpiralMover(Node):
    def __init__(self):
        super().__init__('spiral_mover')
        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)
        self.time = 0.0
        self.max_radius = 5.0  # максимальный радиус спирали
        self.cycle_duration = 60.0  # время полного цикла (в секундах)

    def move(self):
        msg = Twist()
        t = self.time % self.cycle_duration

        # Параметр нормированный от 0 до 1
        norm_t = t / self.cycle_duration

        # Радиус спирали: сначала растёт до max_radius, потом уменьшается обратно
        if norm_t < 0.5:
            radius = 2 * norm_t * self.max_radius
        else:
            radius = 2 * (1 - norm_t) * self.max_radius

        # Угловая скорость
        angular_speed = 0.5

        # Линейная скорость зависит от радиуса и угловой скорости
        linear_speed = radius * angular_speed

        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        self.cmd_pub.publish(msg)
        self.time += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = SpiralMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
