import rospy
from geometry_msgs.msg import Twist
import time


def move_square(side_length, speed):
    rospy.init_node('turtlesim', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.angular.z = 0

    duration_straight = side_length / speed
    duration_turn = 1.57 / speed

    rate = rospy.Rate(10)
    for i in range(4):
        # Mover em linha reta
        start_time = time.time()
        while time.time() - start_time < duration_straight and not rospy.is_shutdown():
            pub.publish(move_cmd)
            rate.sleep()
        
        # Parar
        move_cmd.linear.x = 0
        pub.publish(move_cmd)
        time.sleep(1)

        # Fazer uma curva
        move_cmd.angular.z = speed
        start_time = time.time()
        while time.time() - start_time < duration_turn and not rospy.is_shutdown():
            pub.publish(move_cmd)
            rate.sleep()

        # Preparar para o próximo segmento de linha reta
        move_cmd.angular.z = 0
        move_cmd.linear.x = speed

        time.sleep(1)



if __name__ == '__main__':
    try:
        move_square(3, 0.5)
    except rospy.ROSInterruptException:
        pass        
