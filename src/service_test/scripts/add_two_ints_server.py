#!/usr/bin/env python3

from    __future__          import print_function
from    service_test.srv    import AddTwoInts, AddTwoIntsResponse
import  rospy

def handle_add_two_ints(request):
    response = request.a + request.b
    print("Returning [%s + %s = %s]" %(request.a, request.b, response))
    return response

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("ready to handle add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()