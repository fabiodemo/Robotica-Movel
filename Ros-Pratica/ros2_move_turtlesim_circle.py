import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reset_simulation = self.create_client(Empty, '/reset_simulation')

    def move_circle(self):
        rate = self.create_rate(10)
        vel = Twist()
        vel.linear.x = 0.5  # Velocidade linear fixa
        vel.angular.z = 1.0  # Velocidade angular para movimento circular
        while rclpy.ok():
            self.pub.publish(vel)
            rate.sleep()

    def reset_turtlebot(self):
        self.get_logger().info('Aguardando serviço de reset...')
        while not self.reset_simulation.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de reset não disponível, esperando novamente...')
        self.get_logger().info('Enviando requisição de reset...')
        reset_request = Empty.Request()
        self.reset_simulation.call_async(reset_request)
        self.get_logger().info('Requisição de reset enviada.')
        time.sleep(5)  # Atraso após o reset

if __name__ == '__main__':
    rclpy.init(args=None)
    circle_mover = CircleMover()

    try:
        circle_mover.reset_turtlebot()
        circle_mover.move_circle()
    except KeyboardInterrupt:
        pass

    circle_mover.destroy_node()
    rclpy.shutdown()
