#!/usr/bin/env python3

import  rospy
import  math
from    std_msgs.msg        import String
from    my_package.msg      import WaypointDistanceRequest
from    my_package.msg      import WaypointDistanceResponse

msg_recived = False
counter     = 0

def callback(msg):
    global msg_recived
    msg_recived = True

def response_callback(msg):
    global counter
    counter = counter + 1

def external_request_callback(msg):
    global counter
    counter = counter + 1

def distance_calculator(P1, P2):
    x1 = P1[0]
    y1 = P1[1]

    x2 = P2[0]
    y2 = P2[1]

    distance = math.pow(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2), 0.5)

    return distance

def listener():
    global msg_recived

    # Init del nodo
    rospy.init_node("test_node_name")
    rate = rospy.Rate(12)

    #  Acquisizione parametri
    x           = rospy.get_param("~x", "0")            # Parametro locale
    n_robot     = rospy.get_param("/n_robot", "1")      # Parametro globale
    my_ns       = rospy.get_namespace()                 # Nome del namespace
    waypoints   = rospy.get_param("/waypoints")

    # ID del robot su cui si trova il nodo
    my_id   = int(my_ns[len(my_ns) - 2])

    # Init lista degli ID degli altri robot
    other_id = []

    #  Dizionario del tipo: ID_robot -> publisher a "/external_request" di quel robot
    external_request_pub = {}

    # Dizionario del tipo: ID_robot -> publisher a "/response" di quel robot
    response_pub = {}

    for id in range(1, n_robot + 1, 1):
        if id is not my_id:
            other_id.append(id)

    # Init subscriber a un topic globale /chat
    sub = rospy.Subscriber("/chat", String, callback, queue_size=1)

    # Init subscirber topic all'interno del mio namespace
    my_response         = rospy.Subscriber("response",          WaypointDistanceResponse,   response_callback,          queue_size=10)
    my_external_request = rospy.Subscriber("external_request",  String,    external_request_callback,  queue_size=10)

    # Init Publisher in tutti i topic del tipo /external_request e /response appartenenti agli altri ns
    for i in range(len(other_id)):

        # Namespace degli altri robot
        prefix = "/robot_" + str(other_id[i])

        external_request_pub[other_id[i]]   = rospy.Publisher(prefix + "/external_request", String, queue_size=10)
        response_pub[other_id[i]]           = rospy.Publisher(prefix + "/response",         WaypointDistanceResponse, queue_size=10)

    dist = distance_calculator([0,0], [waypoints[2]["x"], waypoints[2]["y"]])

    # Test corretta acquisizione parametri
    msg1 = "namespace: " + my_ns + " my_id: " + str(my_id) + " total robot: " + str(n_robot) + " other robots: " + str(other_id)
    msg2 = "waypoint id: " + str(waypoints[2]["id"]) + " dist: " + str(dist)

    # Test custom message
    msg3 = WaypointDistanceResponse()
    msg3.robot_id       = my_id
    msg3.waypoint_id    = 2
    msg3.distance       = dist

    while not rospy.is_shutdown():

        # Quando viene pubblicato un msg su chat, pubblico su tutti i topic nella lista external_request
        if msg_recived:
            for i in range(len(other_id)):
                external_request_pub[other_id[i]].publish(msg1)
                external_request_pub[other_id[i]].publish(msg2)
                
                # msg3.header.stamp = rospy.Time.now()
                response_pub[other_id[i]].publish(msg3)
            msg_recived = False

        # Attendo il periodo assegnato al nodo
        rate.sleep()


if __name__ == '__main__':
    try: 
        listener()
    except rospy.ROSInterruptException:
        pass