#!/usr/bin/env python3

import  rospy
import  actionlib
from    move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal
from    my_package.msg      import WaypointAssigned
from    actionlib_msgs.msg  import GoalStatus
from    std_msgs.msg        import String

def waypoint_callback(msg):
    """
    Callback per la gestione dei waypoint.
    Questo callback viene chiamato quando viene ricevuto un messaggio contenente un ID di waypoint.
    Se l'ID del waypoint è diverso da 0, viene aggiunto alla lista globale `my_waypoints`.
    Se l'ID del waypoint è 0, la variabile globale `enable` viene impostata su True.Questo
    evento indica che tutti i waypoint sono stati ricevuti e il robot può iniziare a muoversi.
    Args:
        msg: Il messaggio ricevuto contenente l'ID del waypoint. Si assume che `msg` abbia un attributo `waypoint_id`.
    """
    global my_waypoints
    global enable

    waypoint_id = msg.waypoint_id

    if waypoint_id != 0:
        my_waypoints.append(str(waypoint_id))
    else:
        enable = True

def goal_handler():
    """
    Gestione dei goal del robot.
    Questo nodo è sempre in ascolto e riceve uno alla volta gli waypoint assegnati dal nodo `waypoint_assignement`.
    Solo dopo aver ricevuto tutti gli waypoint, il robot inizia a muoversi.
    
    Parametri ROS:
    - ~x: Coordinata x della posizione iniziale (default "0").
    - ~y: Coordinata y della posizione iniziale (default "0").
    - /waypoints: Lista di tutti gli waypoints
    Sottoscrizioni:
    - waypoint_assigned: Topic per ricevere i waypoint assegnati.
    Action Client:
    - move_base: Client per inviare goal al server move_base.
    Logica:
    - Se `enable` è True, il robot si muove verso i waypoint specificati.
    - Se `homing` è True, il robot ritorna alla posizione iniziale.
    """
    
    global enable
    global my_waypoints

    # Init del nodo
    rospy.init_node("goal_handler")
    rate = rospy.Rate(30)

    x_0         = rospy.get_param("~x", "0")
    y_0         = rospy.get_param("~y", "0")
    waypoints   = rospy.get_param("/waypoints")
    my_ns       = rospy.get_namespace()                 

    waypoints_sub = rospy.Subscriber("waypoint_assigned", WaypointAssigned, waypoint_callback, queue_size=1)
    done_pub = rospy.Publisher("done", String, queue_size=1)
    
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    enable = False
    homing = False

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

            # Qui il codice si blocca fino a quando il goal non è raggiunto/aborted
            client.wait_for_result()
            result = client.get_result()
            status = client.get_state()
            
            # Check waypoint raggiunto
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo(my_ns + " GOAL HANDLER: goal reached number " + my_waypoints[k])
                # Salvare nodi raggiunti
            else:
                rospy.loginfo(my_ns + " GOAL HANDLER: GOAL NOT REACHED")
                # Salvare nodi non raggiunti
            
            k = k + 1

            if k == len(my_waypoints):
                enable = False
                homing = True

        elif homing is True:
            
            coordinates = [x_0, y_0]
            goal.target_pose.pose.position.x = coordinates[0]
            goal.target_pose.pose.position.y = coordinates[1]
            goal.target_pose.pose.orientation.w = 1
        
            client.send_goal(goal)
            client.wait_for_result()
            
            # QUi il codice si blocca fino a quando il goal non è raggiunto/aborted
            result = client.get_result()
            status = client.get_state()
            
            # Check posizione di home raggiunta
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo(my_ns + " GOAL HANDLER: homing position reached")
            else:
                rospy.loginfo(my_ns + " GOAL HANDLER: FAILED TO REACH HOMING POSITION")
            
            done_msg = String()
            done_msg.data = my_ns + " done"
            done_pub.publish(done_msg)
            rospy.signal_shutdown("Task completed")
        rate.sleep()


if __name__ == '__main__':
    try:
        goal_handler()
    except rospy.ROSInterruptException:
        pass