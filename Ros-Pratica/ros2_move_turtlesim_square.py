import math
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SquareMover(Node):
    def __init__(self, side_length, speed):
        super().__init__('square_mover')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.reset_simulation = self.create_client(Empty, '/reset_simulation')
        self.side_length = side_length
        self.speed = speed
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def euler_from_quaternion(x, y, z, w):
        """
        Converte um quaternion em um ângulo de Euler yaw.
        """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        # Atualiza a posição atual com base na odometria
        position = msg.pose.pose.position
        self.current_x = position.x
        self.current_y = position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_yaw = yaw

    def move_square(self):
        rate = self.create_rate(10)
        vel = Twist()
        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw

        for i in range(4):
            # Mover em linha reta para a distância desejada
            while math.sqrt((self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2) < self.side_length:
                vel.linear.x = self.speed
                vel.angular.z = 0.0
                self.pub.publish(vel)
                rate.sleep()

            vel.linear.x = 0.0
            self.pub.publish(vel)
            time.sleep(1)
            start_x = self.current_x
            start_y = self.current_y

            # Girar
            target_yaw = self.current_yaw + math.pi / 2  # Aproximadamente 90 graus
            while abs(self.current_yaw - target_yaw) > 0.05:  # Margem de erro
                vel.angular.z = self.speed
                self.pub.publish(vel)
                rate.sleep()

            vel.angular.z = 0.0
            self.pub.publish(vel)
            time.sleep(1)
            start_yaw = self.current_yaw

    def reset_turtlebot(self):
        # Aguarda até que o serviço de reset esteja disponível
        self.get_logger().info('Aguardando serviço de reset...')
        while not self.reset_simulation.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de reset não disponível, esperando novamente...')
        self.get_logger().info('Enviando requisição de reset...')
        reset_request = Empty.Request()
        self.reset_simulation.call_async(reset_request)
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