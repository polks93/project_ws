import copy
import numpy as np

def generate_bounding_boxes(global_circles, workspace):
    """
    Genera una lista di bounding box uniti per un insieme di cerchi.
    Questa funzione prende una lista di cerchi, calcola i loro bounding box,
    e unisce i bounding box dei cerchi che si intersecano. Il risultato è una
    lista di bounding box che rappresentano l'area combinata dei cerchi che si
    sovrappongono.
    Args:
        global_circles (list): Una lista di cerchi, dove ogni cerchio è rappresentato
                        da una struttura dati appropriata (ad esempio, un tuple
                        o un oggetto con attributi di coordinate e raggio).
    Returns:
        dict: Un dizionario che rappresenta i bounding box uniti con chiavi 'box' e 'center'.
                'box' è una tupla contenente le coordinate del bounding box (x_min, y_min, x_max, y_max).
                'center' è una tupla contenente le coordinate del centro del bounding box (Cx, Cy).
    """
    
    circles = copy.deepcopy(global_circles) 
    
    bounding_boxes = [get_bounding_box(circle, safe_distance=0.5) for circle in circles]

    merged_boxes = {}
    merged_boxes_id = 0
    while circles:
        # Prendi un cerchio e il suo bounding box
        current_circle = circles.pop(0)
        current_box = bounding_boxes.pop(0)
        
        i = 0
        while i < len(circles):
            if do_circles_intersect(current_circle, circles[i]):
                # Se i cerchi si intersecano, unisci i loro bounding box
                current_box = merge_bounding_boxes(current_box, bounding_boxes.pop(i))
                # Rimuovi il cerchio con cui è stata fatta l'unione
                circles.pop(i)
            else:
                i += 1
        
        # Aggiungi il bounding box unito alla lista dei risultati
        adjusted_box = adjust_bounding_box(current_box, min_size=2, workspace=workspace)
        
        x_min, y_min, x_max, y_max = adjusted_box
        Cx = (x_min + x_max) / 2
        Cy = (y_min + y_max) / 2
        
        merged_boxes[merged_boxes_id] = {'box': adjusted_box, 'center': (Cx, Cy)}
        merged_boxes_id += 1
    
    return merged_boxes

def get_bounding_box(circle, safe_distance):
    """
    Calcola il riquadro di delimitazione di un cerchio.

    Args:
        circle (dict): Un dizionario che rappresenta un cerchio con chiavi 'center' e 'radius'.
                       'center' è un altro dizionario con chiavi 'x' e 'y' che rappresentano le coordinate del centro.
                       'radius' è un valore numerico che rappresenta il raggio del cerchio.

    Returns:
        tuple: Una tupla contenente le coordinate del riquadro di delimitazione (x_min, y_min, x_max, y_max).
    """

    x_min = circle['center']['x'] - circle['radius'] - safe_distance
    x_max = circle['center']['x'] + circle['radius'] + safe_distance
    y_min = circle['center']['y'] - circle['radius'] - safe_distance
    y_max = circle['center']['y'] + circle['radius'] + safe_distance
    
    return (x_min, y_min, x_max, y_max)

def do_circles_intersect(circle1, circle2):
    """
    Verifica se due cerchi si intersecano.

    Args:
        circle1 (dict): Un dizionario che rappresenta un cerchio con chiavi 'center' e 'radius'.
                        'center' è un altro dizionario con chiavi 'x' e 'y' che rappresentano le coordinate del centro.
                        'radius' è un valore numerico che rappresenta il raggio del cerchio.
        circle2 (dict): Un dizionario che rappresenta un cerchio con chiavi 'center' e 'radius'.
                        'center' è un altro dizionario con chiavi 'x' e 'y' che rappresentano le coordinate del centro.
                        'radius' è un valore numerico che rappresenta il raggio del cerchio.

    Returns:
        bool: True se i cerchi si intersecano, False altrimenti.
    """
    center1_x = circle1['center']['x']
    center1_y = circle1['center']['y']
    radius1 = circle1['radius']
    
    center2_x = circle2['center']['x']
    center2_y = circle2['center']['y']
    radius2 = circle2['radius']
    
    # Calcola la distanza tra i centri dei due cerchi
    distance = np.sqrt((center1_x - center2_x) ** 2 + (center1_y - center2_y) ** 2)
    
    # Verifica se i cerchi si intersecano
    return distance < (radius1 + radius2)

def merge_bounding_boxes(box1, box2):
    """
    Unisce due riquadri di delimitazione.

    Args:
        box1 (tuple): Una tupla contenente le coordinate del riquadro di delimitazione (x_min, x_max, y_min, y_max).
        box2 (tuple): Una tupla contenente le coordinate del riquadro di delimitazione (x_min, x_max, y_min, y_max).

    Returns:
        tuple: Una tupla contenente le coordinate del riquadro di delimitazione risultante dall'unione dei due riquadri.
    """
    x_min = min(box1[0], box2[0])
    y_min = min(box1[1], box2[1])
    x_max = max(box1[2], box2[2])
    y_max = max(box1[3], box2[3])
    
    return (x_min, y_min, x_max, y_max)

def adjust_bounding_box(box, min_size, workspace):
    """
    Regola un bounding box per garantire che abbia una dimensione minima specificata e che non esca dal workspace.
    Args:
        box (tuple): Una tupla contenente le coordinate del bounding box (x_min, y_min, x_max, y_max).
        min_size (float): La dimensione minima desiderata per la larghezza e l'altezza del bounding box.
    Returns:
        tuple: Una tupla contenente le nuove coordinate del bounding box (x_min, y_min, x_max, y_max),
               regolate per garantire che la larghezza e l'altezza siano almeno min_size.
    """
    
    x_min_ws, y_min_ws, x_max_ws, y_max_ws = workspace
    x_min, y_min, x_max, y_max = box
    
    # Calcolare la larghezza e l'altezza attuali
    width = x_max - x_min
    height = y_max - y_min
    
    # Calcolare il centro del bounding box
    center_x = (x_max + x_min) / 2.0
    center_y = (y_max + y_min) / 2.0
    
    # Se la larghezza è inferiore a min_size, espandere simmetricamente
    if width < 2:
        x_min = center_x - min_size/2  
        x_max = center_x + min_size/2
    
    # Se l'altezza è inferiore a min_size, espandere simmetricamente
    if height < 2:
        y_min = center_y - min_size/2
        y_max = center_y + min_size/2
    
    # Regola il bounding box per garantire che non esca dal workspace
    if x_min < x_min_ws:
        x_min = x_min_ws
    if y_min < y_min_ws:
        y_min = y_min_ws
    if x_max > x_max_ws:
        x_max = x_max_ws
    if y_max > y_max_ws:
        y_max = y_max_ws
      
    return x_min, y_min, x_max, y_max
