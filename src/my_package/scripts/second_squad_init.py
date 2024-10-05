#!/usr/bin/env python3

import rospy
import numpy as np
import copy
import matplotlib.pyplot as plt
from my_package.msg import BoundingBox, BoundingBoxArray

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

def generate_bounding_boxes(global_circles):
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
        list: Una lista di bounding box uniti. Ogni bounding box è rappresentato
              da una struttura dati appropriata (ad esempio, un tuple o un oggetto
              con attributi di coordinate).
    """
    
    circles = copy.deepcopy(global_circles) 
    
    bounding_boxes = [get_bounding_box(circle, safe_distance=1) for circle in circles]

    merged_boxes = []
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
        adjusted_box = adjust_bounding_box(current_box, min_size=2)
        merged_boxes.append(adjusted_box)
    
    return merged_boxes

def adjust_bounding_box(box, min_size):
    """
    Regola un bounding box per garantire che abbia una dimensione minima specificata.
    Args:
        box (tuple): Una tupla contenente le coordinate del bounding box (x_min, y_min, x_max, y_max).
        min_size (float): La dimensione minima desiderata per la larghezza e l'altezza del bounding box.
    Returns:
        tuple: Una tupla contenente le nuove coordinate del bounding box (x_min, y_min, x_max, y_max),
               regolate per garantire che la larghezza e l'altezza siano almeno min_size.
    """

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
    
    return x_min, y_min, x_max, y_max

def plot_circles_and_boxes(circles, merged_boxes):
    # Plot original circles
    fig, ax = plt.subplots()

    for circle in circles:
        center_x = circle['center']['x']
        center_y = circle['center']['y']
        radius = circle['radius']
        circle_patch = plt.Circle((center_x, center_y), radius, color='b', fill=False)
        ax.add_patch(circle_patch)

    # Plot merged bounding boxes
    for box in merged_boxes:
        x_min, y_min, x_max, y_max = box
        width = x_max - x_min
        height = y_max - y_min
        rectangle = plt.Rectangle((x_min, y_min), width, height, edgecolor='r', facecolor='none')
        ax.add_patch(rectangle)

    ax.set_aspect('equal', 'box')
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.grid(True)
    plt.title("Circles and Merged Bounding Boxes")
    plt.show()
    
def publish_boxes(merged_boxes, pub):
    # Creare il messaggio custom BoundingBoxArray
    box_array_msg = BoundingBoxArray()

    # Aggiungi ogni bounding box alla lista
    for box in merged_boxes:
        x_min, y_min, x_max, y_max = box
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0

        # Creare il messaggio per ogni bounding box
        bbox_msg = BoundingBox(
            x_min=x_min, 
            y_min=y_min, 
            x_max=x_max, 
            y_max=y_max,
            center_x=center_x,
            center_y=center_y
        )
        
        # Generro array contenente tutti i box
        box_array_msg.boxes.append(bbox_msg)

    # Pubblica il messaggio
    pub.publish(box_array_msg)
           
def second_squad_init():
    rospy.init_node('second_squad_init')
    rate = rospy.Rate(30)
    
    # Init publisher
    box_pub = rospy.Publisher('/boxes', BoundingBoxArray, queue_size=10)
    
    # Import dati circles come lista di dizionari
    circles = rospy.get_param("/circles", [])

    # Genera i bounding boxes per i cerchi
    merged_boxes = generate_bounding_boxes(circles)
    # plot_circles_and_boxes(circles, merged_boxes)

    while not rospy.is_shutdown():
        # Pubblica i bounding boxes
        publish_boxes(merged_boxes, box_pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        second_squad_init()
    except rospy.ROSInterruptException:
        pass