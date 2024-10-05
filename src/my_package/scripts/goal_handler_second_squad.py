#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def amcl_callback(msg):
    # Posizione
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    # Quaternione
    orientation_q = msg.pose.pose.orientation
    orientation_z = orientation_q.z
    orientation_w = orientation_q.w

def goal_handler():
    rospy.init_node('goal_handler')
    rate = rospy.Rate(30)
    
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_callback, queue_size=1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        goal_handler()
    except rospy.ROSInterruptException:
        pass