import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareMover(Node):
    def __init__(self, side_length, speed):
        super().__init__('square_mover')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.side_length = side_length
        self.speed = speed

    def move_square(self):
        rate = self.create_rate(10)
        vel = Twist()

        for i in range(4):
            vel.linear.x = self.speed
            vel.angular.z = 0
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).seconds < self.side_length / self.speed:
                self.pub.publish(vel)
                rate.sleep()

            vel.linear.x = 0
            self.pub.publish(vel)
            time.sleep(1)

            vel.angular.z = self.speed
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).seconds < 1.57 / self.speed:
                self.pub.publish(vel)
                rate.sleep()

            vel.angular.z = 0
            self.pub.publish(vel)
            time.sleep(1)

if __name__ == '__main__':
    rclpy.init()
    square_mover = SquareMover(3, 0.5)

    try:
        square_mover.move_square()
    except KeyboardInterrupt:
        pass

    square_mover.destroy_node()
    rclpy.shutdown()