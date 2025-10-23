import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Текущая цель из топика current_target_name
        self.current_target = 'carrot1'

        # TF для трансформов
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Публикация управления движением turtle2
        self.cmd_pub = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

        # Подписка на изменение активной цели
        self.target_sub = self.create_subscription(
            String,
            '/current_target_name',
            self.target_callback,
            10
        )

        # Таймер для контроля движения
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Turtle controller started and listening to /current_target_name')

    def target_callback(self, msg):
        if msg.data != self.current_target:
            self.get_logger().info(f'Switching target to {msg.data}')
            self.current_target = msg.data

    def control_loop(self):
        try:
            if not self.tf_buffer.can_transform('turtle2', self.current_target, rclpy.time.Time()):
                self.get_logger().info(f"Waiting for transform turtle2 to {self.current_target}")
                return

            trans = self.tf_buffer.lookup_transform('turtle2', self.current_target, rclpy.time.Time())

            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            distance = math.sqrt(dx**2 + dy**2)

            angle_to_target = math.atan2(dy, dx)
            twist = Twist()
            twist.linear.x = min(1.0, distance)
            twist.angular.z = angle_to_target
            self.cmd_pub.publish(twist)

            # Можно добавить лог публикации, если нужно
            # self.get_logger().info(f'Moving towards {self.current_target}, distance: {distance:.2f}')

        except TransformException as e:
            self.get_logger().warn(f"Could not transform to {self.current_target}: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
