import rospy
from geometry_msgs.msg import Twist
import sys


def turtle_circle(side_length, speed):
    rospy.init_node('turtlesim', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.angular.z = 0

if __name__ == '__main__':
    try:
        turtle_circle(-3)
    except rospy.ROSInterruptException:
        pass        
