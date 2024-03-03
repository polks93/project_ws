#!/usr/bin/env python3

import  rospy
import  actionlib
from    move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal
from    my_package.msg      import WaypointAssigned

def waypoint_callback(msg):
    global my_waypoints
    global enable

    waypoint_id = msg.waypoint_id

    if waypoint_id != 0:
        my_waypoints.append(str(waypoint_id))
    else:
        enable = True

def goal_handler():
    global enable
    global my_waypoints

    # Init del nodo
    rospy.init_node("goal_handler")
    rate = rospy.Rate(30)
    
    waypoints = rospy.get_param("/waypoints")

    waypoints_sub = rospy.Subscriber("waypoint_assigned", WaypointAssigned, waypoint_callback, queue_size=1)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    enable = False
    my_waypoints = []
    k = 0

    client.wait_for_server()
    while not rospy.is_shutdown():

        if enable is True:

            coordinates = waypoints[my_waypoints[k]]
            goal.target_pose.pose.position.x = coordinates[0]
            goal.target_pose.pose.position.y = coordinates[1]
            goal.target_pose.pose.orientation.w = 1
            
            client.send_goal(goal)

            client.wait_for_result()
            result = client.get_result()

            if result:
                rospy.loginfo("GOAL HANDLER: goal reached number " + my_waypoints[k])
                k = k + 1
            else: 
                rospy.loginfo("moving to goal...")              

            if k == len(my_waypoints):
                enable = False

            rate.sleep()
        else:
            rate.sleep()


if __name__ == '__main__':
    try:
        goal_handler()
    except rospy.ROSInterruptException:
        pass