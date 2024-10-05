#!/usr/bin/env python3

import  rospy
import  numpy                       as np
import  matplotlib.pyplot           as plt
from    std_msgs.msg                import String
from    map_change_detection.msg    import ChangedCells
from    sklearn.cluster             import DBSCAN
from    scipy.spatial               import ConvexHull, QhullError
from    nav_msgs.msg                import OccupancyGrid
from    visualization_msgs.msg      import Marker, MarkerArray
import yaml

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
    """
    Raggruppa i punti in cluster utilizzando l'algoritmo DBSCAN e calcola un cerchio che racchiude ogni cluster.
    Args:
        points (list of list of float): Lista di punti da raggruppare, dove ogni punto è rappresentato come una lista di coordinate.
        eps (float, opzionale): Distanza massima tra due punti per essere considerati nello stesso cluster. Default è 0.5.
        min_samples (int, opzionale): Numero minimo di punti per formare un cluster. Default è 1.
        inflation_factor (float, opzionale): Fattore di inflazione per aumentare il raggio del cerchio. Default è 0.2.
        min_radius (float, opzionale): Raggio minimo del cerchio. Default è 0.025.
    Returns:
        list of dict: Lista di dizionari, ognuno contenente il centro e il raggio di un cerchio che racchiude un cluster.
                      Ogni dizionario ha le chiavi 'center' (tuple di float) e 'radius' (float).
    """
    
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
    
    """
    Callback per il messaggio delle nuove celle occupate. Utilizza una funzione di
    clustering per raggruppali in cerchi.
    Args:
        msg: Messaggio contenente gli indici delle celle occupate.
    Variabili globali:
        map_width (int): Larghezza della mappa.
        map_resolution (float): Risoluzione della mappa.
        map_origin (tuple): Origine della mappa.
        circles (list): Lista di cerchi che racchiudono i punti degli ostacoli.
        circles_created (bool): Flag che indica se i cerchi sono stati creati.
    Funzioni chiamate:
        index_to_coordinates(obstacles_index, map_width, map_resolution, map_origin):
            Converte gli indici delle celle occupate in coordinate.
        cluster_and_enclose_points(points):
            Raggruppa i punti e li racchiude in cerchi.
    Note:
        La funzione aggiorna le variabili globali `circles` e `circles_created` in base agli ostacoli rilevati.
    """
    
    global map_width
    global map_resolution
    global map_origin
    global circles
    global circles_created

    obstacles_index = list(msg.toOcc)

    if map_width > 0 and len(obstacles_index) > 0:
        
        points = index_to_coordinates(obstacles_index, map_width, map_resolution, map_origin)
        circles = cluster_and_enclose_points(points)
        circles_created = True

def gridmap_callback(msg):
    """
    Callback per la gestione del messaggio della mappa a griglia.
    Questo callback viene chiamato quando viene ricevuto un messaggio della mappa a griglia.
    Aggiorna le variabili globali `map_width`, `map_resolution` e `map_origin` con le informazioni
    contenute nel messaggio ricevuto.
    Args:
        msg (nav_msgs/OccupancyGrid): Il messaggio della mappa a griglia ricevuto.
    Log:
        Registra un messaggio di log con il valore della larghezza della mappa ricevuta.
    """
    
    global map_width
    global map_resolution
    global map_origin

    map_width = msg.info.width
    map_resolution = msg.info.resolution
    map_origin = msg.info.origin

    rospy.loginfo(f"Received gridmap with width: {msg.info.width}")

def done_callback(msg):
    """
    Callback eseguito quando viene ricevuto un messaggio di completamento.
    Parametri:
    msg (Message): Il messaggio ricevuto che segnala il completamento.
    
    Imposta la variabile globale `done_first_squad` a True.
    """
    
    global done_first_squad
    done_first_squad = True

def save_circles_to_yaml(circles, file_path):
    """
    Salva i dettagli dei cerchi (centro e raggio) in un file YAML.

    Args:
        circles (list of dict): Lista di cerchi, dove ciascun cerchio è rappresentato come un dizionario con chiavi 'center' e 'radius'.
        file_path (str): Percorso del file YAML dove salvare i dati.
    """
    data = []
    
    # Crea una lista di dizionari con i dettagli di ogni cerchio
    for circle in circles:
        x = float(circle['center'][0])
        y = float(circle['center'][1])
        radius = float(circle['radius'])
        entry = {
            'center': {
                'x': x,  # Coordinata X del centro
                'y': y,  # Coordinata Y del centro
            },
            'radius': radius  # Raggio del cerchio
        }
        data.append(entry)
    
    # Scrivi i dati su un file YAML
    with open(file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)
    
    # Stampa un messaggio di conferma
    rospy.loginfo(f"Cerchi salvati su {file_path}")

def publish_circles_as_markers(circles, pub_marker):
    """
    Pubblica i cerchi come marker RViz.
    Questo metodo crea e pubblica marker di tipo cilindro in RViz per rappresentare i cerchi rilevati.
    Prima di pubblicare i nuovi marker, cancella tutti i marker precedenti.
    Args:
        circles (list): Lista di dizionari, dove ogni dizionario rappresenta un cerchio con chiavi 'center' (coordinate x, y) e 'radius' (raggio).
        pub_marker (rospy.Publisher): Publisher ROS per pubblicare i marker.
    Returns:
        None
    """
    
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
    """
    Funzione principale per il rilevamento degli ostacoli.
    Questa funzione inizializza il nodo ROS, i subscriber e i publisher necessari per il rilevamento degli ostacoli
    utilizzando una mappa di occupazione e le celle cambiate. La funzione rimane in esecuzione fino non viene
    arrestata, pubblicando i cerchi rilevati come marker.
    
    Variabili globali:
    - map_width: Larghezza della mappa di occupazione.
    - map_resolution: Risoluzione della mappa di occupazione.
    - map_origin: Origine della mappa di occupazione.
    - circles: Lista di cerchi rilevati.
    - circles_created: Flag che indica se i cerchi sono stati creati.
    
    Parametri ROS:
    - map_topic: Il topic della mappa di occupazione (default: "/map").
    - cells_topic: Il topic delle celle cambiate (default: "/total_changed_cells").
    - marker_topic: Il topic dei marker dei cerchi (default: "/circles_marker").
    
    Subscribers:
    - sub_map: Subscriber per il topic della mappa di occupazione.
    - sub_cells: Subscriber per il topic delle celle cambiate.
    - sub_done: Subscriber per il topic di completamento della prima squadra
    
    Publishers:
    - pub_marker: Publisher per il topic dei marker dei cerchi.
    La funzione attende fino a quando la larghezza della mappa non è disponibile, quindi annulla la subscription alla mappa.
    Durante l'esecuzione, se i cerchi sono stati creati, li pubblica come marker e resetta il flag circles_created.
    """
    
    global map_width
    global map_resolution
    global map_origin
    global circles
    global circles_created
    global done_first_squad
    
    map_topic = rospy.get_param("map_topic", "/map")
    cells_topic = rospy.get_param("changed_cells_topic", "/total_changed_cells")
    marker_topic = rospy.get_param("marker_topic", "/circles_marker")

    map_width = 0
    circles = []
    circles_created = False
    done_first_squad = False
    
    rospy.init_node("obstacle_detection")
    rate = rospy.Rate(10)
    
    # Subscribers
    sub_map = rospy.Subscriber(map_topic, OccupancyGrid, gridmap_callback, queue_size=1)
    sub_cells = rospy.Subscriber(cells_topic, ChangedCells, cells_callback, queue_size=1)
    sub_done = rospy.Subscriber("/done_first_squad", String, done_callback, queue_size=1)
    
    # Publishers
    pub_marker = rospy.Publisher(marker_topic, MarkerArray, queue_size=10)

    while map_width == 0:
        rospy.loginfo("Waiting for gridmap")
        rate.sleep()

    # Una volta ottenute le informazioni necessarie, unregister la callback per fermare la subscription
    rospy.Subscriber.unregister(sub_map)

    while not rospy.is_shutdown():
        if circles_created is True:
            publish_circles_as_markers(circles, pub_marker)
            circles_created = False
        
        if done_first_squad:
            path = "/home/paolo/project_ws/src/simulation/config/circles.yaml"
            save_circles_to_yaml(circles, path)
            rospy.signal_shutdown("Done")
        rate.sleep()


if __name__ == '__main__':
    try:
        obstacle_detection()
    except rospy.ROSInterruptException:
        pass