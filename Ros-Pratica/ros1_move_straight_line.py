#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('turtlesim', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    vel = Twist()

    distance = 5
    speed = 1.5
    vel.linear.x = abs(0.5)
    vel.linear.y = 0
    vel.linear.x = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    while not rospy.is_shutdown():

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < distance):
            pub.publish(vel)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed*(t1-t0)
        
        vel.linear.x = 0
        pub.publish(vel)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass