#! /usr/bin/env python3

import  rospy
import  math
from    visualization_msgs.msg  import Marker
from    geometry_msgs.msg       import PoseWithCovarianceStamped
from    nav_msgs.msg            import OccupancyGrid
from    sensor_msgs.msg         import LaserScan

def amcl_callback(msg):

    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y


    z = msg.pose.pose.orientation.z
    q0 = msg.pose.pose.orientation.w
    q0 = math.copysign(q0, z)
    theta = 2*math.acos(q0)

def map_callback(msg):

    global my_map
    global map_recived

    map_recived = True
    my_map = msg

def laser_callback(msg):

    global laser_recived
    global laser_scan

    laser_recived = True
    laser_scan = msg

def marker_maker():
    global x, y, theta
    global my_map
    global map_recived
    global laser_recived
    global laser_scan

    rospy.init_node('rviz_marker')
    rate = rospy.Rate(30)

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
    map_pub =    rospy.Publisher("/custom_map", OccupancyGrid, queue_size=1)
    amcl_sub = rospy.Subscriber("/robot_1/amcl_pose", PoseWithCovarianceStamped, amcl_callback, queue_size=1)
    map_sub  = rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1)
    laser_sub = rospy.Subscriber("/robot_1/laser_scan", LaserScan, laser_callback, queue_size=1)
    
    marker = Marker()

    marker.header.frame_id = "map"

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
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

    k = 0
    x = 0
    y = 0
    theta = 0
    theta_0 = - 3.14159
    laser_recived = False
    map_recived = False
    my_map = OccupancyGrid()

    while not rospy.is_shutdown():


        if map_recived:
            cell_x = round((x - my_map.info.origin.position.x)/my_map.info.resolution)
            cell_y = round((y - my_map.info.origin.position.y)/my_map.info.resolution)

            index = cell_x + cell_y*my_map.info.width

            buffer = list(my_map.data)
            buffer[index] = 100
            buffer[0] = 100
            buffer[703] = 100
            my_map.data = tuple(buffer)

        if laser_recived:
            laser_recived = False
            x_beam = []
            y_beam = []
            for i in range(len(laser_scan.ranges)):

                x_beam.append(x + laser_scan.ranges[i]*math.cos(theta_0 + i*math.pi/180 + theta))
                y_beam.append(y + laser_scan.ranges[i]*math.sin(theta_0 + i*math.pi/180 + theta))

            marker.pose.position.x = x_beam[180]
            marker.pose.position.y = y_beam[180]

        marker_pub.publish(marker)
        map_pub.publish(my_map)
        rate.sleep()
        
        
        
if __name__ == '__main__':
    try:
        marker_maker()
    except rospy.ROSInterruptException:
        pass