#!/usr/bin/env python3

from    __future__          import print_function
from service_test.srv import DistanceRequest, DistanceRequestResponse
import math
import rospy


waypoint_dic = {1: [0,0], 2: [1,1], 3: [2,2]}
my_position = [0,0]

def handle_distance_calc(request):

    id = request.waypoint_id

    x = waypoint_dic[id][0]
    y = waypoint_dic[id][1]
    response = math.pow(math.pow(x - my_position[0], 2) + math.pow(y - my_position[1], 2), 0.5)
    return response

def distance_calc_server():
    rospy.init_node('distance_calc_server')
    service = rospy.Service('distance_calculator', DistanceRequest, handle_distance_calc)
    print("ready to handle distance calc")
    rospy.spin()

if __name__ == "__main__":
    distance_calc_server()