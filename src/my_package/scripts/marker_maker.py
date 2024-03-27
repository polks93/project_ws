#! /usr/bin/env python3

import  rospy
from    visualization_msgs.msg  import Marker
from    geometry_msgs.msg       import PoseWithCovarianceStamped
from    nav_msgs.msg            import OccupancyGrid

def amcl_callback(msg):
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

def map_callback(msg):
    global my_map
    
    buffer = list(msg.data)
    buffer[0] = 100
    my_map = msg

    my_map.data = tuple(buffer)
    prova = buffer[0]
    rospy.loginfo(str(prova))
def marker_maker():
    global x, y
    global my_map
    global map_pub

    rospy.init_node('rviz_marker')
    rate = rospy.Rate(30)

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
    map_pub =    rospy.Publisher("/custom_map", OccupancyGrid, queue_size=1)
    amcl_sub = rospy.Subscriber("/robot_1/amcl_pose", PoseWithCovarianceStamped, amcl_callback, queue_size=1)
    map_sub  = rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1)
    
    
    marker = Marker()

    marker.header.frame_id = "map"

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.001

    # Set the color
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    x = 0
    y = 0


    my_map = OccupancyGrid()
    # my_map.header.frame_id = "custom_map"
    # my_map.info.resolution = 0.05
    # my_map.info.width = 2
    # my_map.info.height = 2
    # my_map.info.origin.position.x = 0
    # my_map.info.origin.position.y = 0
    # my_map.info.origin.position.z = 0
    # my_map.info.origin.orientation.x = 0
    # my_map.info.origin.orientation.y = 0
    # my_map.info.origin.orientation.z = 0
    # my_map.info.origin.orientation.w = 1
    # my_map.data[0] = -1


    while not rospy.is_shutdown():
        marker.pose.position.x = x
        marker.pose.position.y = y

        marker_pub.publish(marker)
        map_pub.publish(my_map)
        rate.sleep()
if __name__ == '__main__':
    try:
        marker_maker()
    except rospy.ROSInterruptException:
        pass