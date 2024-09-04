#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, TransformStamped
import tf
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

"""Converte le coordinate della gridmap (riga, colonna) in coordinate nel mondo (x, y)."""
def grid_to_world(occupancy_grid, row, col):
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin

    # La colonna rappresenta l'asse x, la riga rappresenta l'asse y
    world_x = origin.position.x + col * resolution
    world_y = origin.position.y + row * resolution

    return world_x, world_y

"""Converte le coordinate del mondo (x, y) in coordinate della gridmap (riga, colonna)."""
def world_to_grid(occupancy_grid, world_x, world_y):
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin

    # La colonna rappresenta l'asse x, la riga rappresenta l'asse y
    col = int((world_x - origin.position.x) / resolution)
    row = int((world_y - origin.position.y) / resolution)

    return row, col

def cell_to_index(cell, map):
    return cell[0] * map.info.width + cell[1]

"""Pubblica un messaggio TF per la posa del LiDAR."""
def publish_tf(lidar_pose):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "lidar_frame"

    t.transform.translation.x = lidar_pose.position.x
    t.transform.translation.y = lidar_pose.position.y
    t.transform.translation.z = lidar_pose.position.z

    quat = tf.transformations.quaternion_from_euler(
        lidar_pose.orientation.x,
        lidar_pose.orientation.y,
        lidar_pose.orientation.z
    )
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)
"""
Simula una scansione LiDAR su una gridmap e genera un messaggio LaserScan.

Args:
- occupancy_grid: Un messaggio ROS nav_msgs/OccupancyGrid che rappresenta la mappa di occupazione
- lidar_pose: Un oggetto Pose che rappresenta la posizione e l'orientamento del LiDAR
- lidar_params: Un dizionario contenente i parametri del LiDAR:
    - "fov": campo visivo in gradi (float)
    - "resolution": risoluzione angolare in gradi (float)
    - "max_range": raggio massimo di scansione (float)

Returns:
- scan_msg: Un messaggio ROS LaserScan che rappresenta la scansione simulata
"""
def simulate_lidar_scan(occupancy_grid, lidar_pose, lidar_params):

    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.array(occupancy_grid.data).reshape((height, width))

    angle_min = -lidar_params['fov'] / 2.0
    angle_max = lidar_params['fov'] / 2.0
    angle_increment = lidar_params['resolution']
    num_readings = int((angle_max - angle_min) / angle_increment) + 1

    ranges = [lidar_params['max_range']] * num_readings

    # Calcola l'orientamento del LiDAR in radianti
    lidar_yaw = np.arctan2(lidar_pose.orientation.z, lidar_pose.orientation.w) * 2

    for i, angle in enumerate(np.arange(angle_min, angle_max, angle_increment)):
        angle_rad = np.deg2rad(angle) + lidar_yaw

        for r in np.arange(0, lidar_params['max_range'], occupancy_grid.info.resolution):
            # Calcola le coordinate del punto nello spazio del mondo
            world_x = lidar_pose.position.x + r * np.cos(angle_rad)
            world_y = lidar_pose.position.y + r * np.sin(angle_rad)
            
            # Converti le coordinate del mondo in coordinate della gridmap (riga, colonna)
            row, col = world_to_grid(occupancy_grid, world_x, world_y)
            
            if row < 0 or row >= height or col < 0 or col >= width:
                break  # Fuori dalla mappa, interrompi il raggio
            
            if data[row, col] > 50:  # consideriamo occupato con soglia > 50
                ranges[i] = r  # Aggiorna la distanza con il valore attuale
                break  # Ostacolo incontrato, interrompi il raggio
    
    # Creazione del messaggio LaserScan
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = "lidar_frame"
    scan_msg.angle_min = np.deg2rad(angle_min)
    scan_msg.angle_max = np.deg2rad(angle_max)
    scan_msg.angle_increment = np.deg2rad(angle_increment)
    scan_msg.time_increment = 0.0  # Tempo tra scansioni successive (non rilevante per simulazione statica)
    scan_msg.range_min = 0.0
    scan_msg.range_max = lidar_params['max_range']
    scan_msg.ranges = ranges

    return scan_msg

def occupancy_grid_callback(msg):
    global map
    global map_recived
    
    map = msg
    map_recived = True
    
def bresenham(cell0, cell1):
    row_0 = cell0[0]
    col_0 = cell0[1]
    row_1 = cell1[0]
    col_1 = cell1[1]   
    
    """Algoritmo di Bresenham per tracciare una linea tra due punti."""
    cells = []
    dx = abs(row_1 - row_0)
    dy = abs(col_1 - col_0)
    sx = 1 if row_0 < row_1 else -1
    sy = 1 if col_0 < col_1 else -1
    err = dx - dy

    while True:
        cells.append((row_0, col_0))
        if row_0 == row_1 and col_0 == col_1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            row_0 += sx
        if e2 < dx:
            err += dx
            col_0 += sy

    return cells

def get_cells_from_ray(occupancy_grid, lidar_pose, angle, range):
    """Restituisce le celle della gridmap viste da un raggio del LiDAR."""
    # Converti la posa del LiDAR in coordinate della gridmap
    lidar_cell = world_to_grid(occupancy_grid, lidar_pose.position.x, lidar_pose.position.y)

    # Calcola il punto finale del raggio in coordinate del mondo
    end_x = lidar_pose.position.x + range * np.cos(angle)
    end_y = lidar_pose.position.y + range * np.sin(angle)

    # Converti il punto finale del raggio in coordinate della gridmap
    end_cell = world_to_grid(occupancy_grid, end_x, end_y)

    # Usa l'algoritmo di Bresenham per ottenere le celle attraversate dal raggio
    cells = bresenham(lidar_cell, end_cell)

    return cells

def get_all_cells_from_lidar(occupancy_grid, lidar_pose, lidar_params, ranges):
    """Restituisce tutte le celle della gridmap viste da tutti i raggi del LiDAR."""
    fov = lidar_params['fov']
    resolution = lidar_params['resolution']

    angle_min = -np.deg2rad(fov / 2)
    angle_max = np.deg2rad(fov / 2)
    angle_increment = np.deg2rad(resolution)
    ray_index = int((angle_max - angle_min) / angle_increment)
    all_cells = set()

    angle = angle_min
    while angle <= angle_max:
        cells = get_cells_from_ray(occupancy_grid, lidar_pose, angle, ranges[ray_index])
        all_cells.update(cells)
        angle += angle_increment

    return all_cells

def create_markers_msg(occupancy_grid, all_cells):
    """Crea un messaggio MarkerArray a partire dalle celle visibili."""
    marker_array = MarkerArray()
    for i, cell in enumerate(all_cells):
        cell_x, cell_y = grid_to_world(occupancy_grid, cell[0], cell[1])  # Converti da [riga, colonna] a coordinate del mondo
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "visible_cells"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = cell_x
        marker.pose.position.y = cell_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Dimensione del cubo
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Opacità
        marker.color.r = 1.0  # Colore rosso
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
    return marker_array
    
""" Funzione eseguita dal main """
def test_gridmap():
    global map
    global map_recived
    
    rospy.init_node("test_gridmap")
    rate = rospy.Rate(30)
    
    # Init flag map_recived
    map_recived = False
    
    # Definisci la posa del LiDAR
    lidar_pose = Pose()
    lidar_pose.position.x = 2.0
    lidar_pose.position.y = 2.0
    lidar_pose.position.z = 0.0
    lidar_pose.orientation.x = 0.0
    lidar_pose.orientation.y = 0.0  
    lidar_pose.orientation.z = 0.0  # nessuna rotazione
    lidar_pose.orientation.w = 1.0
    
    # Definisci i parametri del LiDAR
    lidar_params = {
        'fov': 359.0,  # 360 gradi di campo visivo
        'resolution': 1.0,  # 1 grado di risoluzione angolare
        'max_range': 3.0  # 10 metri di raggio massimo
    }

    # Publisher per il messaggio LaserScan simulato
    laser_scan_publisher = rospy.Publisher('/simulated_scan', LaserScan, queue_size=10)
    visible_cells_publisher = rospy.Publisher('/visible_cells', MarkerArray, queue_size=10)

    # Subscriber per la OccupancyGrid, passando anche i parametri del LiDAR e la sua posa
    rospy.Subscriber('/map', OccupancyGrid, occupancy_grid_callback, queue_size=1)

    
    while not rospy.is_shutdown():
        publish_tf(lidar_pose)
        
        if map_recived:
            scan_msg = simulate_lidar_scan(map, lidar_pose, lidar_params)
            ranges = scan_msg.ranges
            
            visible_cells = get_all_cells_from_lidar(map, lidar_pose, lidar_params, ranges)
            visible_cells_msg = create_markers_msg(map, visible_cells)
            
            cells_index = []
            for cell in visible_cells:
                cells_index.append(cell_to_index(cell, map))
            rospy.loginfo(cells_index)
            
            laser_scan_publisher.publish(scan_msg)
            visible_cells_publisher.publish(visible_cells_msg)

        rate.sleep()
        
        
if __name__ == '__main__':
    try:
        test_gridmap()
    except rospy.ROSInterruptException:
        pass
