#!/usr/bin/env python3

import  rospy
from    std_msgs.msg import String
from    std_msgs.msg import Int64MultiArray
from    std_msgs.msg import Int64
from    service_test.srv import *


def callback(msg): 
    global response
    global msg_recived
    x = msg.data[0]
    y = msg.data[1]
    rospy.wait_for_service('add_two_ints', timeout=10)

    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        response.data = add_two_ints(x, y).sum
        msg_recived = True

    except rospy.ServiceException:
        rospy.logerr("Service call failed")


def test_client():
    global response
    global msg_recived

    rospy.init_node("test_client_node")
    rate = rospy.Rate(10)
    sub = rospy.Subscriber("/request", Int64MultiArray, callback, queue_size=1)
    pub = rospy.Publisher("/response", Int64, queue_size=1)
    counter = rospy.Publisher("/counter", Int64, queue_size=10)
    msg_recived = False
    response = Int64()
    x = Int64()
    x.data = 0
    while not rospy.is_shutdown():
        counter.publish(x)
        x.data = x.data + 1
        if msg_recived is True:
            pub.publish(response)
            msg_recived = False

        rate.sleep()

if __name__ == "__main__":
    try:
        test_client()
    except rospy.ROSInterruptException:
        pass