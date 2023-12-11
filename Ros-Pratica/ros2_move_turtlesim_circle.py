import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty 


class SquareMover(Node):
    def __init__(self, side_length, speed):
        super().__init__('square_mover')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.side_length = side_length
        self.speed = speed
        self.reset_client = self.create_client(Empty, '/reset_simulation')

    def move_square(self):
        rate = self.create_rate(10)
        vel = Twist()

        for i in range(4):
            vel.linear.x = self.speed
            vel.angular.z = 0.0
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < self.side_length / self.speed:
                self.pub.publish(vel)
                rate.sleep()

            vel.linear.x = 0.0
            self.pub.publish(vel)
            time.sleep(1)

            vel.angular.z = self.speed
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 1.57 / self.speed:
                self.pub.publish(vel)
                rate.sleep()

            vel.angular.z = 0.0
            self.pub.publish(vel)
            time.sleep(1)

    def reset_turtlebot(self):
        # Aguarda até que o serviço de reset esteja disponível
        self.get_logger().info('Aguardando serviço de reset...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de reset não disponível, esperando novamente...')
        self.get_logger().info('Enviando requisição de reset...')
        reset_request = Empty.Request()
        self.reset_client.call_async(reset_request)
        self.get_logger().info('Requisição de reset enviada.')

if __name__ == '__main__':
    rclpy.init()
    square_mover = SquareMover(0.5, 0.2)

    try:
        square_mover.reset_turtlebot()
        square_mover.move_square()
    except KeyboardInterrupt:
        pass

    square_mover.destroy_node()
    rclpy.shutdown()