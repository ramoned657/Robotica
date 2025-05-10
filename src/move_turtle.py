#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def mover_tortuga():
    rospy.init_node('mover_tortuga', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz

    cmd = Twist()
    cmd.linear.x = 1.0    # velocidad hacia adelante
    cmd.angular.z = 0.5   # velocidad de giro

    while not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover_tortuga()
    except rospy.ROSInterruptException:
        pass
