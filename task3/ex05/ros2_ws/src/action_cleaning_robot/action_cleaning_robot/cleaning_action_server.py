#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.action import ActionServer
import math

from action_cleaning_robot.action import CleaningTask

class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'CleaningTask',
            self.execute_callback)
        self.pose = None
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.get_logger().info('Cleaning Action Server started')

    def pose_callback(self, msg):
        self.pose = msg

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def execute_callback(self, goal_handle):
        # Запоминаем стартовую точку
        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        start_x = self.pose.x
        start_y = self.pose.y
        start_theta = self.pose.theta

        twist = Twist()
        linear_speed = 1.0
        angular_speed_gain = 2.0
        strip_step = 0.03

        def turn_to_angle(target_theta):
            while rclpy.ok():
                angle_diff = normalize_angle(target_theta - self.pose.theta)
                if abs(angle_diff) > 0.01:
                    twist.linear.x = 0.0
                    twist.angular.z = angular_speed_gain * angle_diff
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    break
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.01)

        def move_to_point(target_x, target_y):
            dx = target_x - self.pose.x
            dy = target_y - self.pose.y
            target_theta = math.atan2(dy, dx)
            turn_to_angle(target_theta)
            while rclpy.ok():
                dist = math.hypot(self.pose.x - target_x, self.pose.y - target_y)
                if dist > 0.015:
                    direction = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
                    angle_diff = normalize_angle(direction - self.pose.theta)
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed_gain * angle_diff
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    break
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.01)

        cleaned_points = 0
        size = goal_handle.request.area_size
        task_type = goal_handle.request.task_type
        result = CleaningTask.Result()

        if task_type == "clean_square":
            if size <= 0.1:
                goal_handle.abort()
                self.get_logger().info('Invalid square size')
                result.success = False
                return result

            num_strips = int(size / strip_step)
            for i in range(num_strips + 1):
                y = start_y + i * strip_step
                if y > start_y + size:
                    break
                # На четной полосе вправо, на нечетной — влево
                if i % 2 == 0:
                    move_to_point(start_x + size, y)
                else:
                    move_to_point(start_x, y)
                cleaned_points += 1
                progress_percent = int((i + 1) / (num_strips + 1) * 100)
                goal_handle.publish_feedback(CleaningTask.Feedback(
                    progress_percent=progress_percent,
                    current_cleaned_points=cleaned_points,
                    current_x=self.pose.x,
                    current_y=self.pose.y
                ))

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            # ДОбАВЛЕН автоматический возврат домой:
            move_to_point(start_x, start_y)
            turn_to_angle(start_theta)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            result.success = True
            result.cleaned_points = cleaned_points
            result.total_distance = size * num_strips
            goal_handle.succeed()
            return result

        elif task_type == "return_home":
            home_x = goal_handle.request.target_x
            home_y = goal_handle.request.target_y
            move_to_point(home_x, home_y)
            turn_to_angle(start_theta)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            result.success = True
            result.cleaned_points = 0
            result.total_distance = math.hypot(self.pose.x - home_x, self.pose.y - home_y)
            goal_handle.succeed()

            goal_handle.publish_feedback(CleaningTask.Feedback(
                progress_percent=100,
                current_cleaned_points=0,
                current_x=self.pose.x,
                current_y=self.pose.y
            ))

            return result

        else:
            self.get_logger().info('Unknown task_type')
            goal_handle.abort()
            result.success = False
            return result

def main(args=None):
    rclpy.init(args=args)
    server = CleaningActionServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
