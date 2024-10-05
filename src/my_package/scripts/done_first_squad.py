#!/usr/bin/env python3

import  rospy
from    std_msgs.msg        import String

def done_callback(msg):
    global dones_dict
    
    # Estrai l'ID del robot dal messaggio ricevuto
    try:
        robot_id = int(msg.data[7])  # Assumendo che l'ID sia nella settima posizione
        dones_dict[robot_id] = True
    except ValueError:
        rospy.logwarn("Impossibile estrarre l'ID del robot dal messaggio: " + msg.data)

def done_first_squad():
    global dones_dict
    
    # Init del nodo
    rospy.init_node("done_first_squad")
    rate = rospy.Rate(30)
    n_robot         = rospy.get_param("/n_robot", "1")      

    dones_dict = {}
    namespaces = []
    sub = []

    for i in range(1, n_robot+1):
        namespaces.append("/robot_"+str(i))
        dones_dict[i] = False
        
    for ns in namespaces:
        sub.append(rospy.Subscriber(ns+"/done", String, done_callback, queue_size=1))
    
    pub = rospy.Publisher("/done_first_squad", String, queue_size=10)
    msg = String()
    msg.data = "done"
    
    while not rospy.is_shutdown():
        
        # Se tutti i robot della prima squadra hanno completato il task
        if all(dones_dict.values()):
            # Pubblica il messaggio e chiudo il nodo
            pub.publish(msg)
            rospy.signal_shutdown("Task prima squadra completato")

        rate.sleep()

if __name__ == '__main__':
    try:
        done_first_squad()
    except rospy.ROSInterruptException:
        pass