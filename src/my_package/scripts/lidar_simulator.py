#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import tf2_ros
from sensor_msgs.msg        import LaserScan
from nav_msgs.msg           import OccupancyGrid
from geometry_msgs.msg      import Pose, TransformStamped
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

"""Converte le coordinate della cella (riga, colonna) in un indice della gridmap."""
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

    # quat = tf.transformations.quaternion_from_euler(
    #     lidar_pose.orientation.x,
    #     lidar_pose.orientation.y,
    #     lidar_pose.orientation.z
    # )
    quat = (
        lidar_pose.orientation.x,
        lidar_pose.orientation.y,
        lidar_pose.orientation.z,
        lidar_pose.orientation.w,
    )
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

""" Simula una scansione LiDAR su una gridmap e genera un messaggio LaserScan."""
def simulate_lidar_scan(occupancy_grid, lidar_pose, lidar_params):
    """ Args:
    - occupancy_grid: Un messaggio ROS nav_msgs/OccupancyGrid che rappresenta la mappa di occupazione
    - lidar_pose: Un oggetto Pose che rappresenta la posizione e l'orientamento del LiDAR
    - lidar_params: Un dizionario contenente i parametri del LiDAR:
        - "fov": campo visivo in gradi (float)
        - "resolution": risoluzione angolare in gradi (float)
        - "max_range": raggio massimo di scansione (float)

    Returns:
    - scan_msg: Un messaggio ROS LaserScan che rappresenta la scansione simulata del LiDAR """
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.array(occupancy_grid.data).reshape((height, width))

    # angle_min = -lidar_params['fov'] / 2.0
    # angle_max = lidar_params['fov'] / 2.0
    angle_min = - 179
    angle_max = 180
    angle_increment = lidar_params['resolution']
    num_readings = int((angle_max - angle_min) / angle_increment) + 1

    ranges = [lidar_params['max_range']] * num_readings
    angles = [0] * num_readings
    
    quaternion = (
        lidar_pose.orientation.x,
        lidar_pose.orientation.y,
        lidar_pose.orientation.z,
        lidar_pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    lidar_yaw = euler[2]  # Angolo di rotazione su z
    # Calcola l'orientamento del LiDAR in radianti
    # lidar_yaw = np.arctan2(lidar_pose.orientation.z, lidar_pose.orientation.w) * 2
    angles_scan = np.linspace(angle_min, angle_max, num_readings)
    
    for i, angle in enumerate(angles_scan):
        angle_rad = np.deg2rad(angle) + lidar_yaw
        
        angles[i] = np.deg2rad(angle) + lidar_yaw
        for r in np.arange(0, lidar_params['max_range'], occupancy_grid.info.resolution/100):
            # Calcola le coordinate del punto nello spazio del mondo
            world_x = lidar_pose.position.x + r * np.cos(angle_rad)
            world_y = lidar_pose.position.y + r * np.sin(angle_rad)
            
            # Converti le coordinate del mondo in coordinate della gridmap (riga, colonna)
            row, col = world_to_grid(occupancy_grid, world_x, world_y)
            
            if row < 0 or row >= height or col < 0 or col >= width:
                break  # Fuori dalla mappa, interrompi il raggio
            
            if data[row, col] > 50 or data[row, col] == -1:  # consideriamo occupato con soglia > 50
                ranges[i] = r + occupancy_grid.info.resolution/2  # Aggiorna la distanza con il valore attuale
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

    return scan_msg, ranges, angles

def simulate_lidar_scan2(occupancy_grid, lidar_pose, lidar_parameters):
    """
    Simula un LiDAR in una gridmap, restituendo un messaggio LaserScan.
    
    Args:
    - occupancy_grid: Un messaggio ROS nav_msgs/OccupancyGrid che rappresenta la mappa di occupazione.
    - lidar_pose: Un oggetto Pose che rappresenta la posizione e l'orientamento del LiDAR.
    - lidar_parameters: Lista con 3 parametri [num_raggi, risoluzione, range_max].

    Returns:
    - scan_msg: Un messaggio ROS LaserScan che rappresenta la scansione simulata del LiDAR.
    """
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.array(occupancy_grid.data).reshape((height, width))

    # Estrazione dei parametri del LiDAR
    num_raggi = 360
    risoluzione = 1
    range_max = 3.0

    angle_min = - (num_raggi * risoluzione) / 2.0
    angle_max = (num_raggi * risoluzione) / 2.0
    angle_increment = risoluzione

    ranges = [range_max] * num_raggi
    
    # Converti l'orientamento del LiDAR in angolo Euleriano (yaw)
    quaternion = (
        lidar_pose.orientation.x,
        lidar_pose.orientation.y,
        lidar_pose.orientation.z,
        lidar_pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    lidar_yaw = euler[2]  # Angolo di rotazione su z (yaw)

    # Calcoliamo gli angoli corretti per ogni raggio usando np.linspace
    angles_scan = np.linspace(angle_min, angle_max, num_raggi)
    angles = 0 * np.ones(num_raggi)
    for i, angle in enumerate(angles_scan):
        angle_rad = np.deg2rad(angle) + lidar_yaw
        angles[i] = angle_rad
        # Simulazione del raggio per ogni angolo
        for r in np.arange(0, range_max, occupancy_grid.info.resolution / 100):
            # Calcola le coordinate del punto nello spazio del mondo
            world_x = lidar_pose.position.x + r * np.cos(angle_rad)
            world_y = lidar_pose.position.y + r * np.sin(angle_rad)
            
            # Converti le coordinate del mondo in coordinate della gridmap (riga, colonna)
            row, col = world_to_grid(occupancy_grid, world_x, world_y)
            
            if row < 0 or row >= height or col < 0 or col >= width:
                break  # Fuori dalla mappa, interrompi il raggio
            
            # Se la cella è occupata o sconosciuta (threshold > 50 o -1)
            if data[row, col] > 50 or data[row, col] == -1:
                ranges[i] = r  # Aggiorna la distanza del raggio
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
    scan_msg.range_max = range_max
    scan_msg.ranges = ranges

    return scan_msg, ranges, angles

def occupancy_grid_callback(msg):
    global map
    global map_recived
    
    map = msg
    map_recived = True

def bresenham(punto1, punto2, map):
    """
    Implementazione dell'algoritmo di Bresenham per calcolare le celle attraversate tra due punti in coordinate world.
    La conversione da world a grid viene eseguita progressivamente.
    
    Input: 
        - punto1, punto2: coordinate nel sistema world
        - gridmap_resolution: risoluzione della griglia
        - gridmap_origin: origine della griglia
    Output: 
        - Lista di celle attraversate
    """
    end_cell = world_to_grid(map, punto2[0], punto2[1])
    cells = set()
    cells.add(end_cell)
    
    resolution = map.info.resolution
    origin = map.info.origin
    
    # Coordinate iniziali e finali del segmento nel sistema "world"
    x0, y0 = punto1
    x1, y1 = punto2
    
    # Differenze tra i punti
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    # Direzione del movimento
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy
    
    # Bresenham modificato per operare su punti world
    while True:
        # Convertiamo il punto corrente dal sistema world alla griglia
        cell = world_to_grid(map, x0, y0)
        cells.add(cell)
        # Se siamo arrivati al punto finale, usciamo dal ciclo
        if round(x0, 1) == round(x1, 1) and round(y0, 1) == round(y1, 1):
            break

        # Passi dell'algoritmo di Bresenham
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx * resolution/20  # Avanza nella direzione x nel mondo
        if e2 < dx:
            err += dx
            y0 += sy * resolution/20  # Avanza nella direzione y nel mondo
    
    return list(cells) 

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
    # cells = bresenham(lidar_cell, end_cell)
    cells = bresenham((lidar_pose.position.x, lidar_pose.position.y), (end_x, end_y), occupancy_grid)
    return cells

def get_all_cells_from_lidar(occupancy_grid, lidar_pose, angles, ranges):
    """Restituisce tutte le celle della gridmap viste da tutti i raggi del LiDAR."""
    all_cells = set()
    for i in range(len(angles)):
        cells = get_cells_from_ray(occupancy_grid, lidar_pose, angles[i], ranges[i])
        all_cells.update(cells)
    
    return all_cells

def create_markers_msg(occupancy_grid, all_cells, color=[1.0, 0.0, 0.0]):
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
        marker.pose.position.x = cell_x + occupancy_grid.info.resolution / 2.0
        marker.pose.position.y = cell_y + occupancy_grid.info.resolution / 2.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = occupancy_grid.info.resolution
        marker.scale.y = occupancy_grid.info.resolution
        marker.scale.z = occupancy_grid.info.resolution
        marker.color.a = 0.4  # Opacità
        marker.color.r = color[0]  
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker_array.markers.append(marker)
    return marker_array

""" Funzione che trova gli 8 vicini di una cella"""
def find_neighbors(cell, map):
    neighbors = []
    row, col = cell
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            if row + i < 0 or row + i >= map.info.height or col + j < 0 or col + j >= map.info.width:
                continue
            neighbors.append((row + i, col + j))
    return neighbors

"""Funzione che trova le celle di frontiera"""
def find_frontier_cells(map, visible_cells):
    frontier_cells = set()
    for cell in visible_cells:
        # Cerco le celle libere in visible_cells
        if map.data[cell_to_index(cell, map)] == 0:
            neighbors = find_neighbors(cell, map)
            # Cerco i vicini delle celle libere che siano di occupazione sconosciuta
            for neighbor in neighbors:
                if map.data[cell_to_index(neighbor, map)] == -1:
                    frontier_cells.add(cell)
                    continue
            
    return frontier_cells

"""Funzione che trova le celle di contorno"""
def find_contour_cells(map, visible_cells):
    contour_cells = set()
    
    for cell in visible_cells:
        free = False
        occupied = False
        # Cerco le celle sconosciute in visible_cells
        if map.data[cell_to_index(cell, map)] == -1:
            neighbors = find_neighbors(cell, map)
            
            # Cerco i vicini delle celle libere che siano di occupazione sconosciuta
            for neighbor in neighbors:
                if map.data[cell_to_index(neighbor, map)] == 0:
                    free = True
                elif map.data[cell_to_index(neighbor, map)] > 50:
                    occupied = True
            
            if free and occupied:
                contour_cells.add(cell)
                
    return contour_cells

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
    lidar_pose.position.x = 8.0
    lidar_pose.position.y = 1.0
    lidar_pose.position.z = 0.0
    lidar_pose.orientation.x = 0.0
    lidar_pose.orientation.y = 0.0  
    lidar_pose.orientation.z = 0.0 # nessuna rotazione
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
    frontier_cells_publisher = rospy.Publisher('/frontier_cells', MarkerArray, queue_size=10)
    contour_cells_publisher = rospy.Publisher('/contour_cells', MarkerArray, queue_size=10)
    
    # Subscriber per la OccupancyGrid, passando anche i parametri del LiDAR e la sua posa
    rospy.Subscriber('/map', OccupancyGrid, occupancy_grid_callback, queue_size=1)

    
    while not rospy.is_shutdown():
        publish_tf(lidar_pose)
        
        if map_recived:
            ray = 269
            scan_msg, ranges, angles = simulate_lidar_scan(map, lidar_pose, lidar_params)
            
            visible_cells = get_all_cells_from_lidar(map, lidar_pose, angles, ranges)
            frontier_cells = find_frontier_cells(map, visible_cells)
            contour_cells = find_contour_cells(map, visible_cells)
            
            visible_cells_msg = create_markers_msg(map, visible_cells, color=[1.0, 0.0, 0.0])
            frontier_cells_msg = create_markers_msg(map, frontier_cells, color=[0.0, 1.0, 0.0])
            contour_cells_msg = create_markers_msg(map, contour_cells, color=[0.0, 0.0, 1.0])          
            
            laser_scan_publisher.publish(scan_msg)
            visible_cells_publisher.publish(visible_cells_msg)
            frontier_cells_publisher.publish(frontier_cells_msg)
            contour_cells_publisher.publish(contour_cells_msg)
        rate.sleep()
 
              
if __name__ == '__main__':
    try:
        test_gridmap()
    except rospy.ROSInterruptException:
        pass
