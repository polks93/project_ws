import numpy as np

""" Classe che si dovrebbe interfacciare correttamente con ROS """

class OccupancyGridWrapper:
    def __init__(self, occupancy_grid):
        """
        Inizializza il wrapper per un oggetto OccupancyGrid di ROS.
        
        Args:
        - occupancy_grid: Messaggio di tipo OccupancyGrid proveniente da ROS.
        """
        self.grid = occupancy_grid
        self.resolution = occupancy_grid.info.resolution
        self.origin = [occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y]
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.data = np.array(occupancy_grid.data).reshape((self.height, self.width))

    def world_to_grid(self, position):
        """
        Converte coordinate mondo (x, y) in coordinate griglia (i, j).
        
        Args:
        - x, y: Coordinate mondo.
        
        Returns:
        - [i, j]: Coordinate della griglia.
        """
        x, y = position
        j = int((x - self.origin[0]) / self.resolution)
        i = int((y - self.origin[1]) / self.resolution)
        return [i, j]

    def grid_to_world(self, cell):
        """
        Converte coordinate griglia (i, j) in coordinate mondo (x, y).
        
        Args:
        - i, j: Coordinate griglia.
        
        Returns:
        - [x, y]: Coordinate mondo del CENTRO della cella.
        """
        i, j = cell
        x = self.origin[0] + (j + 0.5) * self.resolution
        y = self.origin[1] + (i + 0.5) * self.resolution
        return [x, y]

    def get_cell(self, cell):
        """
        Ottiene il valore di una cella nella matrice di occupazione.
        
        Args:
        - i, j: Coordinate della cella nella griglia.
        
        Returns:
        - Valore della cella (0 = libero, 1 = occupato, -1 = sconosciuto).
        """
        i, j = cell
        if 0 <= i < self.height and 0 <= j < self.width:
            return self.data[i, j]
        else:
            return None  # Coordinate fuori dalla mappa

    def set_cell(self, cell, value):
        """
        Imposta il valore di una cella nella matrice di occupazione.
        
        Args:
        - i, j: Coordinate della cella nella griglia.
        - value: Valore da impostare (0 = libero, 1 = occupato, -1 = sconosciuto).
        """
        i, j = cell
        if 0 <= i < self.height and 0 <= j < self.width:
            self.data[i, j] = value
            # Aggiornare i dati nell'OccupancyGrid ROS
            
            self.update_ros_grid([i, j], value)
            

    def update_ros_grid(self, cell, value):
        i, j = cell
        buffer = list(self.grid.data)
        buffer[i * self.width + j] = value
        self.grid.data = tuple(buffer)


def lidar_raycast(pose, lidar_params, map):
    """
    Simula un raycast LIDAR su una mappa di occupazione.
    
    Args:
    - pose: Posizione (x, y, theta) del robot.
    - lidar_params: Parametri del LIDAR (numero di raggi, risoluzione angolare, raggio massimo).
    - map: Oggetto della classe OccupancyGridWrapper.
    
    Returns:
    - angles: Angoli dei raggi LIDAR.
    - ranges: Distanze misurate per ciascun raggio.
    - visible_cells: Celle visibili colpite dai raggi.
    """
    # OccupancyGrid contenuta nel wrapped
    data = map.data
    
    # Converto la posizione del robot da mondo a griglia
    x0, y0, theta0 = pose
    cell0 = map.world_to_grid([x0, y0])
    # x0, y0 = map.grid_to_world(cell0)
    
    # Import parametri utili lidar
    ray_num = lidar_params['ray_num']
    resolution = lidar_params['resolution']
    max_range = lidar_params['max_range']
    
    # Init array per gli angoli e le distanze
    angles = np.zeros([ray_num])
    ranges = max_range * np.ones([ray_num])
    
    # Se la cella iniziale è occupata o sconosciuta, interrompo il raycast
    if map.get_cell(cell0) == -1 or map.get_cell(cell0) == 100:
        return angles, ranges, None

    # Init set di celle contenenti valori unici delle celle visibili
    visible_cells = set()
    
    # Simulazione del raycast per ciascun raggio
    for i in range(ray_num):
        # Calcolo l'angolo assouluto del raggio i-esimo
        angle = np.deg2rad(i * resolution + theta0)
        
        # Salvo il valore dell'angolo assoluto in gradi
        angles[i] = np.rad2deg(angle)
        
        # Calcola l'avanzamento del raggio fino al raggio massimo
        for r in np.arange(0, max_range, 0.01):
            x = x0 + r * np.cos(angle)
            y = y0 + r * np.sin(angle)
            
            # Converte le coordinate mondo in coordinate griglia
            cell = map.world_to_grid([x, y])
            
            # Controlla se il raggio è uscito dai limiti della griglia
            if cell[0] >= data.shape[0] or cell[1] >= data.shape[1] or cell[0] < 0 or cell[1] < 0:
                break
            
            # Se incontro una cella occupata o sconosciuta, interrompo il raggio
            elif map.get_cell(cell) == 100 or map.get_cell(cell) == -1:
                ranges[i] = r
                visible_cells.add(tuple(cell))
                break
            
            # Altrimenti, aggiungi la cella come visibile
            else:
                visible_cells.add(tuple(cell))

    visible_cells = np.array(list(visible_cells))
    return angles, ranges, visible_cells