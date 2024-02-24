#!/usr/bin/env python3
import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from    std_msgs.msg        import String
import math

def callback(msg):
    global enable
    enable = True

#  Funzione per calcoalre la distanza euclidea tra 2 punti
def distance_calculator(P1, P2):

    x1 = P1[0]
    y1 = P1[1]

    x2 = P2[0]
    y2 = P2[1]

    distance = math.pow(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2), 0.5)
    return distance

def distance_from_path(path):
    k = 0
    length = len(path)
    cumulative_distance = 0
    x0 = path[0].pose.position.x
    y0 = path[0].pose.position.y
    P0 = [x0, y0]

    for k in range(0, length - 1, 1):
        x1 = path[k].pose.position.x
        y1 = path[k].pose.position.y
        P1 = [x1, y1]
        cumulative_distance = cumulative_distance + distance_calculator(P0, P1)
        P0 = P1

    return cumulative_distance

def call_makeplan():
    global enable
    # Initialize ROS node
    rospy.init_node('make_plan_client')
    rate = rospy.Rate(30)

    # Create service client
    client = rospy.ServiceProxy('move_base/make_plan', GetPlan)
    sub = rospy.Subscriber("/chat2", String, callback, queue_size=1)
    # Create request message
    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    start.pose.orientation.w = 1.0
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 10.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0
    tolerance = 0.1

    enable = False
    while not rospy.is_shutdown():
        if enable is True:
            # Call service
            try:
                response = client(start, goal, tolerance)
                # Print response message
                rospy.loginfo("Got plan with %d waypoints.", len(response.plan.poses))


                path = response.plan.poses
                distance = distance_from_path(path)
                rospy.loginfo("distance is: %f", distance)

                enable = False
                # Print the pose and orientation of the first waypoint
                # if response.plan.poses:
                #     first_waypoint = response.plan.poses[0]
                #     rospy.loginfo("First waypoint pose: %f, %f, %f",
                #                 first_waypoint.pose.position.x,
                #                 first_waypoint.pose.position.y,
                #                 first_waypoint.pose.position.z)
                #     rospy.loginfo("First waypoint orientation: %f, %f, %f, %f",
                #                 first_waypoint.pose.orientation.w,
                #                 first_waypoint.pose.orientation.x,
                #                 first_waypoint.pose.orientation.y,
                #                 first_waypoint.pose.orientation.z)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service move_base/make_plan: %s", e)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        call_makeplan()
    except rospy.ROSInterruptException:
        pass