#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from service_full_name.srv import SummFullName

class ClientName(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(SummFullName, 'SummFullName')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Сервис еще не доступен, ожидаем...')
        self.req = SummFullName.Request()

    def send_request(self, last_name, name, first_name):
        self.req.last_name = last_name
        self.req.name = name
        self.req.first_name = first_name
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    client = ClientName()

    if len(sys.argv) != 4:
        client.get_logger().info('Пожалуйста, укажите 3 аргумента: фамилия имя отчество')
        sys.exit(1)

    client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(f'Ошибка вызова сервиса: {e}')
            else:
                client.get_logger().info(f'Полное имя: {response.full_name}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
