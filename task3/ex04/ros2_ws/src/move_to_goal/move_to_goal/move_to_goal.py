import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import sys

class MoveToGoal(Node):
    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal')
        
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta
        self.pose = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.get_logger().info(f'Move to goal: x={goal_x}, y={goal_y}, theta={goal_theta}')
        
    def pose_callback(self, msg):
        self.pose = msg
        
    def angle_diff(self, target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
        
    def move_turtle(self):
        if self.pose is None:
            return
        
        # Параметры управления
        distance_tolerance = 0.1
        angle_tolerance = 0.05
        max_linear_speed = 2.0
        max_angular_speed = 4.0
        
        # Расстояние до цели
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Угол до цели
        path_angle = math.atan2(dy, dx)
        angle_to_goal = self.angle_diff(path_angle, self.pose.theta)
        
        twist = Twist()
        
        if distance > distance_tolerance:
            # Поворот в сторону цели
            if abs(angle_to_goal) > angle_tolerance:
                twist.angular.z = max_angular_speed * (angle_to_goal / abs(angle_to_goal))
                twist.linear.x = 0.0
            else:
                # Движение вперёд, корректируя угол
                twist.linear.x = min(max_linear_speed, distance)
                twist.angular.z = max_angular_speed * angle_to_goal
        else:
            # Если в точке, поворачиваемся к нужному углу theta
            angle_remain = self.angle_diff(self.goal_theta, self.pose.theta)
            if abs(angle_remain) > angle_tolerance:
                twist.angular.z = max_angular_speed * (angle_remain / abs(angle_remain))
                twist.linear.x = 0.0
            else:
                # Цель достигнута - остановка
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Goal reached')
                self.timer.cancel()
                
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        print('Usage: ros2 run move_to_goal move_to_goal x y theta')
        return
    
    try:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_theta = float(sys.argv[3])
    except ValueError:
        print('Invalid input parameters, must be floats')
        return
    
    node = MoveToGoal(goal_x, goal_y, goal_theta)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
