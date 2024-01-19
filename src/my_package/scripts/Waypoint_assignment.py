#!/usr/bin/env python3

import  rospy
from    std_msgs.msg        import String

msg_recived = False

def callback(msg):
    global msg_recived
    msg_recived = True

def listener():
    global msg_recived

    # Init del nodo
    rospy.init_node("test_node_name")

    #  Acquisizione parametri dal launch file
    x       = rospy.get_param("~x", "0")
    n_robot = rospy.get_param("/n_robot", "1")
    my_ns   = rospy.get_namespace()

    # ID del robot
    my_id   = int(my_ns[len(my_ns) - 2])

    # Lista degli ID degli altri robot
    other_id = []

    #  Dizionario del tipo: ID_robot -> publisher a external_request di quel robot
    external_request = {}

    for id in range(1, n_robot + 1, 1):
        if id is not my_id:
            other_id.append(id)

    sub = rospy.Subscriber("/chat", String, callback, queue_size=1)

    for i in range(len(other_id)):
        topic = "/robot_" + str(other_id[i]) + "/external_request"
        external_request[other_id[i]] = rospy.Publisher(topic, String, queue_size=1)

    msg = "namespace: " + my_ns + " my_id: " + str(my_id) + " total robot: " + str(n_robot) + " other robots: " + str(other_id)

    rate = rospy.Rate(12)

    while not rospy.is_shutdown():
        if msg_recived:
            for i in range(len(other_id)):
                external_request[other_id[i]].publish(msg)
            msg_recived = False

        rate.sleep()

if __name__ == '__main__':
    try: 
        listener()
    except rospy.ROSInterruptException:
        pass