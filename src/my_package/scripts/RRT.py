#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from gridmap_functions import lidar_raycast
from lidar_simulator_v1 import find_frontier_cells, find_contour_cells

class OccMatrix():
    def __init__(self, dim=[10,10], occ_space=None, unknown_space=None, resolution=0.1, origin=[0, 0]):
        """
        Inizializza una matrice di occupazione.
        
        Args:
        - dim: Dimensioni della griglia (larghezza e altezza in metri).
        - free_space: Lista di celle libere (valore = 0).
        - occ_space: Lista di celle occupate (valore = 1).
        - resolution: Risoluzione della mappa (dimensione di una cella in metri).
        - origin: Coordinate mondo dell'origine della mappa (coordinate [0,0] della griglia).
        """
        self.width = int(np.ceil(dim[0]/resolution))
        self.height = int(np.ceil(dim[1]/resolution))
        self.resolution = resolution
        self.origin = origin  # Posizione in coordinate mondo dell'origine (cella [0,0])
        self.shape = [self.height, self.width]  # La forma della griglia (numero di righe, colonne)
        self.data = np.zeros(self.shape, dtype=int)  # Inizializza la griglia con valori liberi
        
        np.random.seed(0)
        
        if unknown_space is not None:
            min_cell = self.world_to_grid([unknown_space[0], unknown_space[1]])
            min_row = min_cell[0]
            min_col = min_cell[1]
            
            max_cell = self.world_to_grid([unknown_space[2], unknown_space[3]])
            max_row = max_cell[0]
            max_col = max_cell[1]
            
            for i in range(min_row, max_row + 1, 1):
                for j in range(min_col, max_col + 1, 1):
                    value = np.random.randint(2) * -1
                    self.set_cell([i, j], value)
                    
        
        if occ_space is not None:
            min_cell = self.world_to_grid([occ_space[0], occ_space[1]])
            min_row = min_cell[0]
            min_col = min_cell[1]
            
            max_cell = self.world_to_grid([occ_space[2], occ_space[3]])
            max_row = max_cell[0]
            max_col = max_cell[1]
            
            for i in range(min_row, max_row + 1, 1):
                for j in range(min_col, max_col + 1, 1):
                    self.set_cell([i, j], 100)
            
    
    def world_to_grid(self, position):
        """
        Converte le coordinate mondo (x, y) in coordinate griglia (riga, colonna).
        
        Args:
        - x, y: Coordinate mondo.
        
        Returns:
        - [riga, colonna]: Coordinate della griglia.
        """
        x, y = position
        colonna = int((x - self.origin[0]) / self.resolution)
        riga = int((y - self.origin[1]) / self.resolution)
        return [riga, colonna]

    def grid_to_world(self, cell):
        """
        Converte le coordinate griglia (riga, colonna) in coordinate mondo (x, y).
        
        Args:
        - i, j: Coordinate griglia.
        
        Returns:
        - [x, y]: Coordinate mondo del centro della cella.
        """
        i, j = cell
        # Calcola le coordinate del centro della cella (i, j)
        x = self.origin[0] + (j + 0.5) * self.resolution
        y = self.origin[1] + (i + 0.5) * self.resolution
        return [x, y]

    def set_cell(self, cell, value):
        """
        Imposta il valore di una cella nella matrice di occupazione.
        
        Args:
        - riga, colonna: Coordinate della cella nella griglia.
        - value: Valore da impostare (0 = libero, 1 = occupato, -1 = sconosciuto).
        """
        riga, colonna = cell
        if 0 <= riga < self.height and 0 <= colonna < self.width:
            self.data[riga, colonna] = value

    def get_cell(self, cell):
        """
        Ottiene il valore di una cella nella matrice di occupazione.
        
        Args:
        - riga, colonna: Coordinate della cella nella griglia.
        
        Returns:
        - Valore della cella (0 = libero, 1 = occupato, -1 = sconosciuto).
        """
        riga, colonna = cell
        if 0 <= riga < self.height and 0 <= colonna < self.width:
            return self.data[riga, colonna]
        else:
            return None  # Coordinate fuori dalla mappa

def update_tree(tree, state, parent, path):
    """
    Aggiorna l'albero RRT con un nuovo nodo.
    Args:
        tree (dict): Dizionario che rappresenta l'albero RRT. Le chiavi sono indici dei nodi e i valori sono dizionari con informazioni sul nodo.
        state (tuple): Stato del nuovo nodo da aggiungere all'albero.
        parent (int): Indice del nodo genitore nel dizionario dell'albero.
        path (list): Lista di tuple che rappresentano il percorso dal nodo genitore al nuovo nodo.
    Returns:
        None
    """
    
    N = len(tree)
    cost = 0
    if len(path) > 0:
        curr_position = path[0][:2]
        
        for i in range(1, len(path)):
            next_position = path[i][:2]
            cost += np.linalg.norm(curr_position - next_position)
            curr_position = next_position
        
        
    tree[N] = {'state': state, 'parent': parent, 'cost': cost, 'path': path}

def generate_random_state(R, curr_state, workspace, map):
    """
    Genera uno stato casuale all'interno di un raggio specificato attorno allo stato corrente.
    Args:
        R (float): Il raggio entro il quale generare lo stato casuale.
        curr_state (array-like): Lo stato corrente rappresentato come una coppia di coordinate (x, y).
        workspace (object): L'oggetto che rappresenta lo spazio di lavoro, utilizzato per verificare i confini.
        map (object): L'oggetto mappa che fornisce metodi per convertire coordinate e controllare celle.
    Returns:
        numpy.ndarray: Un array contenente le coordinate (x, y) dello stato casuale generato.
    """

    Cx = curr_state[0]
    Cy = curr_state[1]
    
    sample_found = False

    while not sample_found:
        alpha = 2 * np.pi * np.random.rand()
        r = R * np.sqrt(np.random.rand())
        x = r * np.cos(alpha) + Cx
        y = r * np.sin(alpha) + Cy
        
        if boundary_check(workspace, x, y):
            cell = map.world_to_grid([x,y])
            
            if map.get_cell(cell) == 0:
                sample_found = True

    return np.array([x, y])


def find_nearest_node(sample, tree):
    """
    Trova il nodo più vicino in un albero di ricerca RRT.
    Args:
        sample (numpy.ndarray): Un array numpy che rappresenta il campione di stato (coordinate x, y, ...).
        tree (list of dict): Una lista di dizionari che rappresenta l'albero di ricerca RRT. Ogni dizionario contiene almeno una chiave 'state' che rappresenta lo stato del nodo (coordinate x, y, ...).
    Returns:
        tuple: Una tupla contenente l'indice del nodo più vicino e lo stato del nodo più vicino.
    """

    N = len(tree)
    distances = np.zeros(N)
    for i in range(N):
        node_state = tree[i]['state']
        distances[i] = np.linalg.norm(sample[:2] - node_state[:2])
    nearest_id = int(np.argmin(distances))
    nearest_state = tree[nearest_id]['state']
    
    return nearest_id, nearest_state

def wrapToPi(angle):
    """
    Converte l'angolo di input nell'intervallo [-pi, pi].
    Parametri:
        angle (float): L'angolo di input da convertire in radianti
    Ritorna:
        float: L'angolo convertito nell'intervallo [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def boundary_check(workspace, x, y):
    """
    Verifica se un punto (x, y) si trova all'interno dei confini specificati del workspace.
    Args:
        workspace (tuple): Una tupla di quattro valori (xmin, ymin, xmax, ymax) che definiscono i confini del workspace.
        x (float): Coordinata x del punto da verificare.
        y (float): Coordinata y del punto da verificare.
    Returns:
        bool: True se il punto (x, y) si trova all'interno dei confini del workspace, altrimenti False.
    """
    
    if workspace[0] <= x <= workspace[2] and workspace[1] <= y <= workspace[3]:
        return True
    return False
    
def generate_new_node(nearest_state, sample, T_max, unicycle_params, map, workspace):
    """
    Genera un nuovo nodo a partire dallo stato più vicino e un campione.
    Args:
        nearest_state (tuple): Lo stato più vicino sotto forma di (x, y, theta).
        sample (tuple): Il campione target sotto forma di (x, y).
        T_max (float): Il tempo massimo per l'integrazione.
        unicycle_params (dict): Parametri del modello unicycle, contenente:
            - 'v_max' (float): Velocità massima.
            - 'omega_max' (float): Velocità angolare massima.
            - 'K_omega' (float): Guadagno per il controllo della velocità angolare.
            - 'dt' (float): Passo di tempo per l'integrazione.
        map (object): Oggetto mappa che fornisce metodi per la conversione e il controllo delle celle.
        workspace (object): Oggetto workspace che definisce i confini dell'area di lavoro.
    Returns:
        tuple: Una tupla contenente:
            - new_node (numpy.ndarray): Il nuovo nodo generato sotto forma di (x, y, theta).
            - path_to_node (numpy.ndarray): Il percorso verso il nuovo nodo come array di stati.
            - node_found (bool): Indica se il nodo è stato trovato correttamente.
    """
    
    # Import parametri del modello unicycle
    v_max = unicycle_params['v_max']
    omega_max = unicycle_params['omega_max']
    K_omega = unicycle_params['K_omega']
    dt = unicycle_params['dt']
    min_distance = v_max * dt

    # Estraggo le coordinate del nodo più vicino
    x, y, theta = nearest_state

    # Inizializzo il percorso dal nodo più vicino
    path_to_node = np.array(nearest_state)
    
    # Init flag node_found
    node_found = False
    new_node = nearest_state
    
    # Simulo la dinamica del modello unicycle per un tempo massimo T_max
    for t in np.arange(0, T_max, dt):
        
        # Check minima distanza
        distance_from_node = np.linalg.norm(sample[:2] - np.array([x,y]))
        if distance_from_node <= min_distance:
            new_node = np.array([x, y, theta])
            return new_node, path_to_node, node_found

        # Calcolo la direzione verso il campione
        theta_des = np.arctan2(sample[1] - y, sample[0] - x)
        theta_diff = wrapToPi(theta - theta_des)
        
        # Controllo proporzionale della velocità angolare
        omega = - K_omega * theta_diff
        # Limito la velocità angolare
        if omega > omega_max:
            omega = omega_max
        elif omega < - omega_max:
            omega = - omega_max

        # Update stato
        x += v_max*np.cos(theta)*dt
        y += v_max*np.sin(theta)*dt
        theta = wrapToPi(theta + omega*dt)

        # Check path fuori dai confini workspace
        if not boundary_check(workspace, x, y):
            return np.array([]), np.array([]), False
        
        # Check collisione con cella occupata o sconosciuta  
        cell = map.world_to_grid([x, y])
        if map.get_cell(cell) == -1 or map.get_cell(cell) == 100:
            return np.array([]), np.array([]), False

        # Aggiorno il percorso al nuovo stato
        path_to_node = np.vstack([path_to_node, np.array([x, y, theta]) ])
        node_found = True

    # Se il percorso è stato completato, restituisci il nuovo nodo
    new_node = np.array([x, y, theta])
    return new_node, path_to_node, node_found

def find_total_path(tree, node_id):
    """
    Trova il percorso totale dal punto iniziale dell'albero a un nodo specifico.
    Args:
        tree (dict): Dizionario che rappresenta l'albero di ricerca. Ogni chiave è un ID di nodo e il valore è un 
                     dizionario con le chiavi 'parent' (ID del nodo genitore) e 'path' (percorso dal nodo genitore 
                     al nodo corrente).
        node_id (int): ID del nodo di destinazione per il quale trovare il percorso totale.
    Returns:
        np.ndarray: Un array numpy che rappresenta il percorso totale dal nodo radice al nodo specificato.
    """
    
    total_path = np.array([])
    
    while node_id != 0:
        parent_id = tree[node_id]['parent']
        path_to_node = tree[node_id]['path']
        
        if len(total_path) == 0:
            total_path = path_to_node
        else:
            total_path = np.vstack([path_to_node, total_path])
        node_id = parent_id

    return total_path

def evaluate_sample(sample, map, weights, lidar_params)-> float:
    """
    Valuta un campione dato utilizzando una funzione di valore basata su celle di frontiera e di contorno.
    Args:
        sample (tuple): Coordinate del campione da valutare.
        map (object): Mappa dell'ambiente in cui si trova il campione.
        weights (dict): Dizionario contenente i pesi per le celle di frontiera e di contorno.
            - 'w_frontier' (float): Peso per le celle di frontiera.
            - 'w_contour' (float): Peso per le celle di contorno.
        lidar_params (dict): Parametri per la simulazione del LIDAR.
    Returns:
        float: Valore calcolato per il campione basato sulla funzione di valore.
    """
    
    w_frontier = weights['w_frontier']
    w_contour = weights['w_contour']
    
    angles, ranges, visible_cells = lidar_raycast(sample, lidar_params, map)
    
    if np.min(ranges) < 0.5:
        return -1
    
    frontier_cells = find_frontier_cells(visible_cells=visible_cells, map=map)
    contour_cells = find_contour_cells(visible_cells=visible_cells, map=map)
    
    N_frontier = len(frontier_cells)
    N_contour = len(contour_cells)
    value_function = w_frontier * N_frontier + w_contour * N_contour
    
    return value_function

def generate_RRT_samples(workspace, map, curr_pose, sampling_params, unicycle_params):
    """
    Genera campioni per l'algoritmo RRT.
    Args:
        workspace (list): Lista di quattro valori [xmin, ymin, xmax, ymax] che definiscono i confini del workspace.
        map (object): Oggetto mappa che fornisce metodi per la conversione e il controllo delle celle.
        curr_pose (numpy.ndarray): Array numpy che rappresenta la posa corrente del robot.
        sampling_params (dict): Dizionario contenente i parametri di campionamento, tra cui:
            - 'R' (float): Raggio massimo per la generazione di campioni.
        unicycle_params (dict): Parametri del modello unicycle, contenente:
            - 'v_max' (float): Velocità massima.
            - 'omega_max' (float): Velocità angolare massima.
            - 'K_omega' (float): Guadagno per il controllo della velocità angolare.
            - 'dt' (float): Passo di tempo per l'integrazione.
    Returns:
        - tree (dict): Dizionario che rappresenta l'albero RRT. Le chiavi sono gli ID dei nodi e i valori sono dizionari con informazioni sul nodo.
        - samples (numpy.ndarray): Un array numpy che contiene i campioni
    """
    # Parametri campionamento
    R           = sampling_params['R']
    N_samples   = sampling_params['N_samples']
    T_max       = sampling_params['T_max']
    
    # Init tree con solo il nodo iniziale
    tree = {}
    update_tree(tree, curr_pose, -1, np.array([]))
    
    # Generazione N_samples campioni
    while len(tree) <= N_samples:
        
        # Genero campione casuale in una cella ammissibile [libera e dentro il workspace]
        sample = generate_random_state(R, curr_pose, workspace, map)
        
        # Cerco il nodo più vicino all'interno dell'albero [distanza euclidea]
        nearest_id, nearest_state = find_nearest_node(sample, tree)
        
        # Cerco di generare un nuovo nodo valido verso il campione
        new_node, path_to_node, node_found = generate_new_node(nearest_state, sample, T_max, unicycle_params, map, workspace)
        
        # Se il nodo è valido, lo aggiungo all'albero
        if node_found:
            update_tree(tree, new_node, nearest_id, path_to_node)
    
    # Elenco dei campioni generati
    samples = np.array([tree[i]['state'] for i in tree])
 
    return tree, samples
    
def main():
    workspace = [0,0,10,10]
    occ_space = [4,4,6,6]
    unknown_space = [5,5,10,10]
    map = OccMatrix(dim=[10,10], occ_space=occ_space, unknown_space=unknown_space)
    
    start_pose = np.array([3.5,5,0])
    
    lidar_params = {'ray_num': 360, 'resolution': 1, 'max_range': 1.0}
    sampling_params = {'R': 3, 'N_samples': 50, 'T_max': 0.5}
    unicycle_params = {'v_max': 1.0, 'omega_max': 5.0, 'K_omega': 2.0, 'dt': 0.01}
    weights = {'w_frontier': 0.1, 'w_contour': 9}
    
    curr_pose = start_pose
    
    valid_samples_found = False
    counter = 0
    
    while not valid_samples_found:
        if counter > 0:
            print('No valid samples found. Retrying...')
            
            unicycle_params['v_max'] = unicycle_params['v_max'] / 2
            unicycle_params['omega_max'] += 0.1
            unicycle_params['K_omega'] += 0.1
            
        tree, samples = generate_RRT_samples(workspace, map, curr_pose, sampling_params, unicycle_params)

        value_function = np.zeros(len(samples))
        
        for i in range(len(samples)):
            value_function[i] = evaluate_sample(samples[i], map, weights, lidar_params)
        
        if np.max(value_function) > 0:
            valid_samples_found = True
            counter += 1
            
    id_best_sample = np.argmax(value_function) 
    best_sample = samples[id_best_sample]
    total_path = find_total_path(tree, id_best_sample)
    
    _, _, visible_cells = lidar_raycast(best_sample, lidar_params, map)
    frontier_cells = find_frontier_cells(visible_cells=visible_cells, map=map)
    contour_cells = find_contour_cells(visible_cells=visible_cells, map=map)
        
    # Creare la colormap: grigio (sconosciuto), bianco (libero), nero (occupato)
    plt.figure(figsize=(20,20))
    cmap = colors.ListedColormap(['gray', 'white', 'black'])
    bounds = [-1.5, -0.5, 0.5, 1.5]  # Limiti per i valori: -1 (sconosciuto), 0 (libero), 1 (occupato)
    norm = colors.BoundaryNorm(bounds, cmap.N)

    # Calcolare l'estensione della mappa in coordinate del mondo
    extent = [
        map.origin[0],  # Estensione asse X: dall'origine fino alla larghezza in metri
        map.origin[0] + map.width * map.resolution,
        map.origin[1],  # Estensione asse Y: dall'origine fino all'altezza in metri
        map.origin[1] + map.height * map.resolution
    ]

    # Plottare la matrice di occupazione
    plt.imshow(map.data, cmap=cmap, norm=norm, origin='lower', extent=extent)
    plt.scatter(start_pose[0], start_pose[1])

    # Plot dell'albero RRT
    for i in range(1, len(tree)):
        path = tree[i]['path']
        x = [sample[0] for sample in path]
        y = [sample[1] for sample in path]
        plt.plot(x, y, color='blue')
    
    # Plot del path verso campione migliore   
    x = [pose[0] for pose in total_path]
    y = [pose[1] for pose in total_path]
    plt.plot(x,y, color='red')
    
    # Plot celle di frontiera e celle di contorno
    for cell in frontier_cells:
        x, y = map.grid_to_world(cell)
        plt.scatter(x, y, color='green')
    
    for cell in contour_cells:
        x, y = map.grid_to_world(cell)
        plt.scatter(x, y, color='orange')
    
    plt.show()    

if __name__ == '__main__':
    main()