#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose

def callback(pose):
    rospy.loginfo("Tortuga en x=%.2f, y=%.2f, θ=%.2f", pose.x, pose.y, pose.theta)

def leer_pose():
    rospy.init_node('leer_pose', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    leer_pose()