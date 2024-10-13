#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from second_squad_functions import generate_bounding_boxes

def init_boxes():
    """ 
    Calcolo i riquadri da esplorare a partire da circles disponibile come parametro globale 
    
    Returns:
        dict: Un dizionario che rappresenta i riquadri da esplorare con chiavi 'box' e 'center'.
                'box' è una tupla contenente le coordinate del riquadro di delimitazione (x_min, y_min, x_max, y_max).
                'center' è una tupla contenente le coordinate del centro del riquadro (Cx, Cy).
    """
    workspace = rospy.get_param("/workspace")
    circles = rospy.get_param("/circles")
    merged_boxes = generate_bounding_boxes(circles, workspace)
    
    return merged_boxes

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

def box_pub_node():
    rospy.init_node('box_array_pub')
    rate = rospy.Rate(1)
    boxes = init_boxes()
    
    # Init publisher
    rect_pub = rospy.Publisher('/rectangle_markers', MarkerArray, queue_size=10)
    point_pub = rospy.Publisher('/point_markers', MarkerArray, queue_size=10)

    
    while not rospy.is_shutdown():
        rect_marker_array = MarkerArray()
        point_marker_array = MarkerArray()
        
        for i, box in boxes.items():
            rect_marker_array.markers.append(create_rectangle_marker(i, box['box']))
            point_marker_array.markers.append(create_point_marker(i, box['center']))
            
        # Pubblica i MarkerArray
        rect_pub.publish(rect_marker_array)
        point_pub.publish(point_marker_array)
        
if __name__ == '__main__':
    try: 
        box_pub_node()
        
    except rospy.ROSInterruptException:
        pass