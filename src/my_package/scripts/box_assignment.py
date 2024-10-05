#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib

from my_package.msg import BoundingBoxArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.srv        import GetPlan
from geometry_msgs.msg   import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal

def amcl_callback(msg):
    # # Posizione
    # x = msg.pose.pose.position.x
    # y = msg.pose.pose.position.y
    
    # # Quaternione
    # orientation_q = msg.pose.pose.orientation
    # orientation_z = orientation_q.z
    # orientation_w = orientation_q.w
    global current_pose
    
    current_pose = (msg.pose.pose)
    
def bounding_boxes_callback(msg):
    """
    Callback per la gestione delle bounding boxes.
    Questo callback viene chiamato quando viene ricevuto un messaggio contenente le bounding boxes.
    Args:
        msg: Il messaggio ricevuto contenente le bounding boxes.
    """
    global bounding_boxes
    global bounding_boxes_recived
      
    if not bounding_boxes_recived:
        # Trasformo il messaggio in un dizionario di dizionari
        bounding_boxes = {}
        for i, box in enumerate(msg.boxes):
            bounding_boxes[i] = {'box': (box.x_min, box.y_min, box.x_max, box.y_max), 'center': (box.center_x, box.center_y)}
        bounding_boxes_recived = True

def create_rectangle_marker(rect_id, rect):
    marker = Marker()
    x_min, y_min, x_max, y_max = rect
    
    # Imposta il tipo di marker (LINE_STRIP per un rettangolo 2D)
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.id = rect_id
    
    # Imposta il frame in cui si trova (per esempio, "map" o "odom")
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # Imposta il colore e la trasparenza del marker (RGBA)
    marker.color.r = 0.0  # Colore blu
    marker.color.g = 1.0  # Colore verde
    marker.color.b = 0.0  # Colore blu
    marker.color.a = 1.0  # Trasparente
    
    # Imposta la scala del marker (spessore della linea)
    marker.scale.x = 0.2  # Spessore della linea

    # Definisci i vertici del rettangolo
    points = []

    # Aggiungi i 4 vertici del rettangolo e chiudi il ciclo tornando al primo punto
    points.append(Point(x_min, y_min, 0))  # Vertice 1
    points.append(Point(x_max, y_min, 0))  # Vertice 2
    points.append(Point(x_max, y_max, 0))  # Vertice 3
    points.append(Point(x_min, y_max, 0))  # Vertice 4
    points.append(Point(x_min, y_min, 0))  # Chiudere il ciclo

    marker.points = points
    
    return marker

def create_point_marker(point_id, point):
    marker = Marker()

    # Imposta il tipo di marker (POINTS per visualizzare un punto)
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.id = point_id
    
    # Imposta il frame in cui si trova (ad esempio, "map" o "odom")
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # Imposta il colore e la trasparenza del punto (RGBA)
    marker.color.r = 0.0  
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Opaco
    
    # Imposta la scala del marker (dimensione del punto)
    marker.scale.x = 0.2  # Larghezza del punto
    marker.scale.y = 0.2  # Altezza del punto

    # Aggiungi il singolo punto alla lista dei punti del marker
    marker.points.append(Point(point[0], point[1], 0))

    return marker

def distance(p1, p2):
    """
    Calcola la distanza euclidea tra due punti 2D.
    Args:
        p1 (tuple): Coordinata del primo punto (x1, y1).
        p2 (tuple): Coordinata del secondo punto (x2, y2).
    Returns:
        float: Distanza euclidea tra i due punti.
    """
    
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_distance_to_rect(start_pose, rect_dict, make_plan_client, safe_distance=0.25):
    
    # Punto iniziale
    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = start_pose.position.x
    start.pose.position.y = start_pose.position.y
    start.pose.orientation.z = start_pose.orientation.z
    start.pose.orientation.w = start_pose.orientation.w

    # Punto finale
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = rect_dict['center'][0]
    goal.pose.position.y = rect_dict['center'][1]
    goal.pose.orientation.w = 1.0
    
    # Tolleranza raggiungimento goal
    tolerance = 0.1

    # Genero il path per raggiungere il centro del rettangolo
    response = make_plan_client(start, goal, tolerance)
    path = response.plan.poses
    path_lenght, goal = truncate_path_at_rectangle(path, rect_dict['box'], safe_distance)
    return path_lenght, goal

def truncate_path_at_rectangle(path, rect, safe_distance):

    x_min, y_min, x_max, y_max = rect
    x_min += safe_distance
    y_min += safe_distance
    x_max -= safe_distance
    y_max -= safe_distance
    
    x0, y0 = path[0].pose.position.x, path[0].pose.position.y
    P0 = (x0, y0)
    
    path_lenght = 0
    for pose_stamped in path:
        # Ottieni le coordinate x e y del punto nel path
        
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        P = (x, y)
        
        path_lenght += distance(P0, P)

        P0 = P
        # Verifica se il punto si trova all'interno dei limiti del rettangolo
        if x_min <= x <= x_max and y_min <= y <= y_max:
            # Se il punto è all'interno del rettangolo, termina l'iterazione
            goal = pose_stamped
            break

    return path_lenght, goal


def box_assignment():
    global bounding_boxes
    global bounding_boxes_recived
    global current_pose
    
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
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_callback, queue_size=1)

    # Init publisher
    rect_pub = rospy.Publisher('/rectangle_markers', MarkerArray, queue_size=10)
    point_pub = rospy.Publisher('/point_markers', MarkerArray, queue_size=10)

    # Init service client
    make_plan_client = rospy.ServiceProxy('move_base/make_plan', GetPlan)
    goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    # Attendo modulo move_base
    make_plan_client.wait_for_service()
    goal_client.wait_for_server()
    
    min_distance = None
    robot_position = (x_0, y_0)
    
    while not rospy.is_shutdown():
        
        if bounding_boxes_recived:
            
            # Creare un MarkerArray per contenere tutti i rettangoli e i punti del centro
            rect_marker_array = MarkerArray()
            point_marker_array = MarkerArray()
            
            if len(bounding_boxes) == 0:
                rospy.loginfo("All boxes have been assigned")
                break
            
            for i, box in bounding_boxes.items():
                rect_marker_array.markers.append(create_rectangle_marker(i, box['box']))
                point_marker_array.markers.append(create_point_marker(i, box['center']))
                
                distance, rect_goal = get_distance_to_rect(current_pose, box, make_plan_client)
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    min_distance_goal = rect_goal
                    rect_id = i
                    
            # Invia il goal al modulo move_base
            rospy.loginfo(rect_id)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = min_distance_goal.pose.position.x
            goal.target_pose.pose.position.y = min_distance_goal.pose.position.y
            goal.target_pose.pose.orientation.z = min_distance_goal.pose.orientation.z
            goal.target_pose.pose.orientation.w = min_distance_goal.pose.orientation.w
            goal_client.send_goal(goal)
            goal_client.wait_for_result()
            
            # Init delle variabili per il prossimo ciclo
            bounding_boxes.pop(rect_id)
            min_distance = None
            
            # Pubblica i MarkerArray
            rect_pub.publish(rect_marker_array)
            point_pub.publish(point_marker_array)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        box_assignment()
    except rospy.ROSInterruptException:
        pass