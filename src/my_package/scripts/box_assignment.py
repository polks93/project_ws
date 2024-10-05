#!/usr/bin/env python3

import rospy
import numpy as np
from my_package.msg import BoundingBoxArray

def bounding_boxes_callback(msg):
    """
    Callback per la gestione delle bounding boxes.
    Questo callback viene chiamato quando viene ricevuto un messaggio contenente le bounding boxes.
    Args:
        msg: Il messaggio ricevuto contenente le bounding boxes.
    """
    global bounding_boxes
    global bounding_boxes_recived
    
    bounding_boxes = msg
    bounding_boxes_recived = True
    
def box_assignment():
    global bounding_boxes
    global bounding_boxes_recived
    
    bounding_boxes_recived = False
    
    rospy.init_node('box_assignment')
    rate = rospy.Rate(30)
    
    #  Acquisizione parametri locali
    x_0             = rospy.get_param("~x", "0")
    y_0             = rospy.get_param("~y", "0")

    # Acquisizione parametri globali
    n_robot         = rospy.get_param("/n_robot", "1") 
    workspace       = rospy.get_param("/workspace")
    
    # Acquisizione namespace  
    my_ns           = rospy.get_namespace()         

    # Init subscriber
    rospy.Subscriber('/boxes', BoundingBoxArray, bounding_boxes_callback, queue_size=10)
    
    while not rospy.is_shutdown():
        if bounding_boxes_recived:
            rospy.loginfo("Bounding boxes recived")
            rospy.loginfo(bounding_boxes)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        box_assignment()
    except rospy.ROSInterruptException:
        pass