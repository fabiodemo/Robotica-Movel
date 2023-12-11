import rclpy
from geometry_msgs.msg import Twist
import time



if __name__ == '__main__':
    try:
        turtle_circle(-3)
    except rospy.ROSInterruptException:
        pass        
