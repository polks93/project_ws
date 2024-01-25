#!/usr/bin/env python3

import  rospy
from    std_msgs.msg import String
from    std_msgs.msg import Float64
from    std_msgs.msg import Int64
from    service_test.srv import AddTwoInts, DistanceRequest

def test_coordination():

    rospy.init_node("test_coordination")
    rate = rospy.Rate(10)

    pub_check = rospy.Publisher("check", Int64, queue_size=1)
    pub_srv = rospy.Publisher("response", Float64, queue_size=1)
    response = Int64()
    response.data = 1
    x = Float64()
    x.data = 0
    while not rospy.is_shutdown():

        try:
            rospy.wait_for_service('distance_calculator', timeout=1)
            response.data = response.data + 1
            distance = rospy.ServiceProxy('distance_calculator', DistanceRequest)
            x.data = distance(2).distance
            pub_srv.publish(x)
        except rospy.exceptions.ROSException:
            response.data = 0

            # try:
            #     add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            #     x.data = add_two_ints(x.data, 1).sum
            #     pub_srv.publish(x)

            # except rospy.ServiceException:
            #     rospy.logerr("error")

        pub_check.publish(response)
        rate.sleep()

if __name__ == "__main__": 
    try: 
        test_coordination()
    except rospy.ROSInterruptException:
        pass