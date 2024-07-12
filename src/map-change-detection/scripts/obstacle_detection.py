#!/usr/bin/env python3

import  rospy
import  numpy                       as np
import  matplotlib.pyplot           as plt

from    map_change_detection.msg    import ChangedCells
from    sklearn.cluster             import DBSCAN
from    scipy.spatial               import ConvexHull, QhullError
from    nav_msgs.msg                import OccupancyGrid
from    visualization_msgs.msg      import Marker, MarkerArray

def index_to_coordinates(indices, width, resolution, origin):
    """
    Args:
    indices (list of int): Lista di indici lineari.
    width (int): Numero di colonne della gridmap.

    Returns:
    list of tuples: Lista di coppie (riga, colonna).
    """
    x0 = origin.position.x
    y0 = origin.position.y

    return [(index % width * resolution + x0 + resolution / 2, 
             index // width * resolution + y0 + resolution / 2) for index in indices]

def cluster_and_enclose_points(points, eps=0.5, min_samples=1, inflation_factor=0.2, min_radius=0.025):
    points_array = np.array(points)
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points_array)
    labels = clustering.labels_
    circles = []
    
    for label in set(labels):
        if label != -1:  # Ignora il rumore identificato da DBSCAN
            cluster_points = points_array[labels == label]
            
            if len(cluster_points) >= 3:
                try:
                    # Tenta di calcolare il Convex Hull solo se ci sono almeno 3 punti non collineari
                    hull = ConvexHull(cluster_points)
                    hull_points = cluster_points[hull.vertices]
                    center = np.mean(hull_points, axis=0)
                    radius = np.max(np.linalg.norm(hull_points - center, axis=1))
                except QhullError:
                    # Se si verifica un errore, usa un approccio più semplice
                    center = np.mean(cluster_points, axis=0)
                    radius = np.max(np.linalg.norm(cluster_points - center, axis=1))
            else:
                # Per meno di 3 punti, il cerchio si basa direttamente sui punti esistenti
                center = np.mean(cluster_points, axis=0)
                radius = np.max(np.linalg.norm(cluster_points - center, axis=1)) if len(cluster_points) > 1 else 0
            
            radius = max(radius * (1 + inflation_factor), min_radius)
            circles.append({'center': tuple(center), 'radius': radius})

    return circles

def cells_callback(msg):
    global map_width
    global map_resolution
    global map_origin
    global circles
    global circles_created

    obstacles_index = list(msg.toOcc)

    if map_width > 0 and len(obstacles_index) > 0:
        points = index_to_coordinates(obstacles_index, map_width, map_resolution, map_origin)
        circles = cluster_and_enclose_points(points)
        
        # Conversione centri e raggi in metri
        # for circle in circles:
        #     center_in_meters = (
        #         map_origin.position.x + circle['center'][0] * map_resolution,
        #         map_origin.position.y + circle['center'][1] * map_resolution
        #     )
        #     radius_in_meters = circle['radius'] * map_resolution
        #     meters_circles.append({
        #         'center': center_in_meters,
        #         'radius': radius_in_meters
        #     })
        circles_created = True
        rospy.loginfo(circles)

def gridmap_callback(msg):
    global map_width
    global map_resolution
    global map_origin

    map_width = msg.info.width
    map_resolution = msg.info.resolution
    map_origin = msg.info.origin

    rospy.loginfo(f"Received gridmap with width: {msg.info.width}")

def publish_circles_as_markers(circles, pub_marker):
    # Crea un Marker per cancellare tutti i marker precedenti
    delete_marker = Marker()
    delete_marker.header.frame_id = "map"
    delete_marker.header.stamp = rospy.Time.now()
    delete_marker.ns = "circles"
    delete_marker.action = Marker.DELETEALL

    # Pubblica il marker di cancellazione
    marker_array = MarkerArray()
    marker_array.markers.append(delete_marker)
    pub_marker.publish(marker_array)

    # Crea e pubblica i nuovi marker
    marker_array = MarkerArray()
    for i, circle in enumerate(circles):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "circles"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = circle['center'][0]
        marker.pose.position.y = circle['center'][1]
        marker.pose.position.z = 0  # Z position of the marker
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 2 * circle['radius']  # Diameter
        marker.scale.y = 2 * circle['radius']  # Diameter
        marker.scale.z = 0.1  # Height of the cylinder
        marker.color.a = 0.5  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0  # Green
        marker.color.b = 0  # Blue
        marker.lifetime = rospy.Duration()  # 0 means infinite lifetime
        marker_array.markers.append(marker)

    pub_marker.publish(marker_array)

def obstacle_detection():
    global map_width
    global map_resolution
    global map_origin
    global circles
    global circles_created

    map_topic = rospy.get_param("map_topic", "/map")
    cells_topic = rospy.get_param("changed_cells_topic", "/total_changed_cells")
    marker_topic = rospy.get_param("marker_topic", "/circles_marker")

    map_width = 0
    circles = []
    circles_created = False

    rospy.init_node("obstacle_detection")
    rate = rospy.Rate(10)
    
    # Subscribers
    sub_map = rospy.Subscriber(map_topic, OccupancyGrid, gridmap_callback, queue_size=1)
    sub_cells = rospy.Subscriber(cells_topic, ChangedCells, cells_callback, queue_size=1)

    # Publishers
    pub_marker = rospy.Publisher(marker_topic, MarkerArray, queue_size=10)

    while map_width == 0:
        rospy.loginfo("Waiting for gridmap")
        rate.sleep()

    # Una volta ottenute le informazioni necessarie, unregister la callback per fermare la subscription
    rospy.Subscriber.unregister(sub_map)

    while not rospy.is_shutdown():
        if circles_created is True:
            rospy.loginfo("Marker visualize")
            rospy.loginfo(str(circles))
            publish_circles_as_markers(circles, pub_marker)
            circles_created = False
        rate.sleep()


if __name__ == '__main__':
    try:
        obstacle_detection()
    except rospy.ROSInterruptException:
        pass