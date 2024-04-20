#!/usr/bin/env python3

import  rospy
from    map_change_detection.msg import ChangedCells

def callback(msg,arg):
    global callback_recived
    global OccBuffer
    global FreeBuffer

    robot_id = arg
    toFree = list(msg.toFree)
    toOcc = list(msg.toOcc)

    OccBuffer = list(set(OccBuffer) | set(toOcc))
    FreeBuffer = list(set(FreeBuffer) | set(toFree))
    callback_recived[robot_id] = True

def changed_cells_union():
    global callback_recived
    global FreeBuffer
    global OccBuffer

    rospy.init_node("changed_cells_union")
    rate = rospy.Rate(20)

    n_robot = rospy.get_param("n_robot", 2)

    robot_namespace = []
    cells_topic_name = []
    subscribers = {}
    robot_id = []
    callback_recived = {}
    OccBuffer = []
    FreeBuffer = []

    for i in range(n_robot):
        robot_namespace.append("robot_" + str(i + 1))
        cells_topic_name.append("/" + robot_namespace[i] + "/map_updater/changed_cells")
        robot_id.append(i+1)

    for i in range(len(robot_id)):
        subscribers[robot_id[i]] = rospy.Subscriber(cells_topic_name[i], ChangedCells, callback, queue_size=1, callback_args=robot_id[i])
        callback_recived[robot_id[i]] = False

    pub = rospy.Publisher("total_changed_cells", ChangedCells, queue_size=1)
    total_changed_cells = ChangedCells()
    while not rospy.is_shutdown():

        for i in range(len(robot_id)):
            if callback_recived[robot_id[i]] is False:
                break

            elif i == n_robot - 1:
                
                for i in range(len(robot_id)):
                    callback_recived[robot_id[i]] = False

                total_changed_cells.toFree = FreeBuffer
                total_changed_cells.toOcc = OccBuffer 
                pub.publish(total_changed_cells)
                total_changed_cells = ChangedCells()
                FreeBuffer = []
                OccBuffer = []

        rate.sleep()

if __name__ == '__main__':
    try:
        changed_cells_union()
    except rospy.ROSInterruptException:
        pass