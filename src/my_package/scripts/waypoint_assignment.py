#!/usr/bin/env python3

import  rospy
import  math
from    std_msgs.msg        import String
from    my_package.msg      import WaypointDistanceRequest
from    my_package.msg      import WaypointDistanceResponse

enable = True
robot_position = [0, 0]

def response_callback(msg):
    counter =  1

def external_request_callback(msg):

    global robot_position
    global waypoints
    global my_id
    global response_pub

    requesting_robot    = msg.robot_id
    waypoint_id         = msg.waypoint_id
    waypoint_position   = waypoints[str(waypoint_id)]
    my_distance         = distance_calculator(robot_position, waypoint_position)

    response_msg = WaypointDistanceResponse()
    response_msg.robot_id       = my_id
    response_msg.waypoint_id    = waypoint_id
    response_msg.distance       = my_distance
    response_pub[requesting_robot].publish(response_msg)

def callback(msg):
    global enable
    enable = True

def distance_calculator(P1, P2):
    x1 = P1[0]
    y1 = P1[1]

    x2 = P2[0]
    y2 = P2[1]

    distance = math.pow(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2), 0.5)

    return distance

def waypoint_assignment():
    global enable
    global robot_position
    global waypoints
    global my_id
    global response_pub

    # Init del nodo
    rospy.init_node("test_node_name")
    rate = rospy.Rate(12)

    #  Acquisizione parametri
    x_0             = rospy.get_param("~x", "0")
    y_0             = rospy.get_param("~y", "0")
    # Y_0             = rospy.get_param("~Y", "0")

    n_robot         = rospy.get_param("/n_robot", "1")      
    my_ns           = rospy.get_namespace()                 
    waypoints       = rospy.get_param("/waypoints_new")

    # ID del robot su cui si trova il nodo
    my_id   = int(my_ns[len(my_ns) - 2])

    # Init varabili
    other_id = []                   # Init lista degli ID degli altri robot
    my_waypoints = []
    waypoints_distance = {}
    external_request_pub = {}       # Dizionario del tipo: ID_robot -> publisher a "/external_request" di quel robot
    response_pub = {}               # Dizionario del tipo: ID_robot -> publisher a "/response" di quel robot

    for id in range(1, n_robot + 1, 1):
        if id is not my_id:
            other_id.append(id)

    sub = rospy.Subscriber("/chat", String, callback, queue_size=1)
    # Init subscirber topic all'interno del mio namespace
    my_response         = rospy.Subscriber("response",          WaypointDistanceResponse,   response_callback,          queue_size=10)
    my_external_request = rospy.Subscriber("external_request",  WaypointDistanceRequest,    external_request_callback,  queue_size=10)

    # Init Publisher in tutti i topic del tipo /external_request e /response appartenenti agli altri ns
    for i in range(len(other_id)):

        # Namespace degli altri robot
        prefix = "/robot_" + str(other_id[i])

        external_request_pub[other_id[i]]   = rospy.Publisher(prefix + "/external_request", WaypointDistanceRequest,    queue_size=10)
        response_pub[other_id[i]]           = rospy.Publisher(prefix + "/response",         WaypointDistanceResponse,   queue_size=10)

    msg = WaypointDistanceResponse()
    msg.robot_id = my_id
    robot_position = [x_0, y_0]
    
    request_msg = WaypointDistanceRequest()
    request_msg.robot_id = my_id

    while not rospy.is_shutdown():
        #  Numero waypoint ancora disponibili
        n_waypoints = len(waypoints)

        if enable is True:

            # Ciclo for che calcola la distanza da tutti gli waypoint
            for i in range(n_waypoints):
                # list() fornisce una lista di tutte le chiavi presenti nel dizionario 'waypoint'
                id_waypoint = list(waypoints)[i]

                # Dizionario: ID waypoint -> distanza dal waypoint
                waypoints_distance[id_waypoint] = distance_calculator(robot_position, waypoints[id_waypoint])
            
            # Lista ordinata del dizionario creato sopra del tipo [['id', distanza], ...]
            waypoint_orderd = sorted(waypoints_distance.items(), key=lambda item:item[1])

            request_msg.waypoint_id = int(waypoint_orderd[0][0])

            for i in range(len(other_id)):
                external_request_pub[other_id[i]].publish(request_msg)


            # msg.waypoint_id = int(waypoint_orderd[1][0])
            # msg.distance = waypoint_orderd[1][1]
            # response_pub[2].publish(msg)

            enable = False
        rate.sleep()

if __name__ == '__main__':
    try: 
        waypoint_assignment()
    except rospy.ROSInterruptException:
        pass