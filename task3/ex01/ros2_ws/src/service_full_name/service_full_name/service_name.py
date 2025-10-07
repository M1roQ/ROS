#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from service_full_name.srv import SummFullName

class ServiceName(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.handle_summ_full_name)

    def handle_summ_full_name(self, request, response):
        full_name = f"{request.last_name} {request.name} {request.first_name}"
        self.get_logger().info(f"Received: {request.last_name}, {request.name}, {request.first_name}")
        response.full_name = full_name
        self.get_logger().info(f"Sending back: {full_name}")
        return response


def main(args=None):
    rclpy.init(args=args)
    service = ServiceName()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
