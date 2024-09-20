#!/usr/bin/env python3

import numpy as np
import rospy

from gridmap_functions      import OccupancyGridWrapper, lidar_raycast
from nav_msgs.msg           import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

def occupancy_grid_callback(msg):
    global map
    global map_recived
    
    map = msg
    map_recived = True

""" Funzione che trova gli 8 vicini di una cella"""
def find_neighbors(cell, map):
    neighbors = []
    row, col = cell
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            if row + i < 0 or row + i >= map.height or col + j < 0 or col + j >= map.width:
                continue
            neighbors.append((row + i, col + j))
    return neighbors

"""Funzione che trova le celle di frontiera"""
def find_frontier_cells(map, visible_cells):
    frontier_cells = set()
    for cell in visible_cells:
        # Cerco le celle libere in visible_cells
        if map.get_cell(cell) == 0:
            neighbors = find_neighbors(cell, map)
            # Cerco i vicini delle celle libere che siano di occupazione sconosciuta
            for neighbor in neighbors:
                if map.get_cell(neighbor) == -1:
                    frontier_cells.add(tuple(cell))
                    continue
            
    return frontier_cells

"""Funzione che trova le celle di contorno"""
def find_contour_cells(map, visible_cells):
    contour_cells = set()
    
    for cell in visible_cells:
        free = False
        occupied = False
        # Cerco le celle sconosciute in visible_cells
        if map.get_cell(cell) == -1:
            neighbors = find_neighbors(cell, map)
            
            # Cerco i vicini delle celle libere che siano di occupazione sconosciuta
            for neighbor in neighbors:
                if map.get_cell(neighbor) == 0:
                    free = True
                elif map.get_cell(neighbor) > 50:
                    occupied = True
            
            if free and occupied:
                contour_cells.add(tuple(cell))
                
    return contour_cells

def create_markers_msg(occupancy_grid, all_cells, color=[1.0, 0.0, 0.0]):
    """Crea un messaggio MarkerArray a partire dalle celle visibili."""
    marker_array = MarkerArray()
    for i, cell in enumerate(all_cells):
        cell_x, cell_y = occupancy_grid.grid_to_world(cell)  # Converti da [riga, colonna] a coordinate del mondo
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
        marker.scale.x = occupancy_grid.resolution
        marker.scale.y = occupancy_grid.resolution
        marker.scale.z = occupancy_grid.resolution
        marker.color.a = 0.4  # Opacità
        marker.color.r = color[0]  
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker_array.markers.append(marker)
    return marker_array

def lidar_simulator_node():
    global map
    global map_recived
    
    rospy.init_node("lidar_simulator_node")
    rate = rospy.Rate(30)
    
    # Init flag map_recived
    map_recived = False
    
    # Definisci la posa del LiDAR
    x = 0.0
    y = 5.0
    theta = 0.0
    lidar_pose = [x, y, theta]
    
    # Definisco lidar parameters
    lidar_params = {'ray_num': 360, 'resolution': 1, 'max_range': 3.0}
    
    # Subscriber per la OccupancyGrid, passando anche i parametri del LiDAR e la sua posa
    rospy.Subscriber('/map', OccupancyGrid, occupancy_grid_callback, queue_size=10)
    
    # Publisher per i MarkerArray
    visible_cells_publisher     = rospy.Publisher('/visible_cells', MarkerArray, queue_size=10)
    frontier_cells_publisher    = rospy.Publisher('/frontier_cells', MarkerArray, queue_size=10)
    contour_cells_publisher     = rospy.Publisher('/contour_cells', MarkerArray, queue_size=10)
    
    """Test inflation mappa"""
    # my_map_publisher            = rospy.Publisher('/my_map', OccupancyGrid, queue_size=10)
    # map_msg = OccupancyGrid()
    # infalted = False
    while not rospy.is_shutdown():
        
        if map_recived:
            my_map = OccupancyGridWrapper(map)  

            """Test inflation mappa"""
            # if not infalted:
            #     infalted = True
            #     my_map.inflate_gridmap_around_position([12,15], 0.5, 1.0) 
            # map_msg = my_map.grid  
            # my_map_publisher.publish(map_msg)
            
            angles, ranges, visible_cells = lidar_raycast(lidar_pose, lidar_params, my_map)
            frontier_cells = find_frontier_cells(my_map, visible_cells)
            contour_cells = find_contour_cells(my_map, visible_cells)
            
            visible_cells_msg = create_markers_msg(my_map, visible_cells, color=[1.0, 0.0, 0.0])
            frontier_cells_msg = create_markers_msg(my_map, frontier_cells, color=[0.0, 1.0, 0.0])
            contur_cells_msg = create_markers_msg(my_map, contour_cells, color=[0.0, 0.0, 1.0])
            
            visible_cells_publisher.publish(visible_cells_msg)
            frontier_cells_publisher.publish(frontier_cells_msg)
            contour_cells_publisher.publish(contur_cells_msg)
        rate.sleep()
 

if __name__ == '__main__':
    try:
        lidar_simulator_node()
    except rospy.ROSInterruptException:
        pass