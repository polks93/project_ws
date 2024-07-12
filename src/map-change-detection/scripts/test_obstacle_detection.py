#!/usr/bin/env python3

import  rospy
from    map_change_detection.msg    import ChangedCells
import  numpy                       as np
from    sklearn.cluster             import DBSCAN
from    scipy.spatial               import ConvexHull, QhullError
import  matplotlib.pyplot            as plt

def index_to_coordinates(indices, width):
    """
    Converti gli indici lineari in coppie di coordinate (riga, colonna).

    Args:
    indices (list of int): Lista di indici lineari.
    width (int): Numero di colonne della gridmap.

    Returns:
    list of tuples: Lista di coppie (riga, colonna).
    """
    return [(index // width, index % width) for index in indices]

def cluster_and_enclose_points(points, eps=4, min_samples=2, inflation_factor=0.1):
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
            
            radius = radius * (1 + inflation_factor)
            circles.append({'center': tuple(center), 'radius': radius})
    
    return circles


def plot_clusters_and_circles(points, circles):
    fig, ax = plt.subplots()
    # Plotta i punti
    x_coords, y_coords = zip(*points)
    ax.scatter(x_coords, y_coords, color='blue', label='Punti')

    # Plotta le circonferenze
    for circle in circles:
        circle_center = circle['center']
        circle_radius = circle['radius']
        circle_plot = plt.Circle(circle_center, circle_radius, color='red', fill=False, linewidth=2, linestyle='--')
        ax.add_patch(circle_plot)
        # ax.plot(circle_center[0], circle_center[1], 'ro')  # Centro del cerchio

    # Impostazioni del grafico
    ax.set_aspect('equal', adjustable='datalim')
    ax.plot()  # Causa la ricalcolazione dei limiti per includere le circonferenze
    plt.title('Punti e Circonferenze Clusterizzati')
    plt.xlabel('Indice Colonna')
    plt.ylabel('Indice Riga')
    plt.legend()
    plt.show()


# Esempio di utilizzo

# Numero di colonne della gridmap
width = 704  # Ad esempio, ipotizziamo 400 colonne

# Esempio di indici
indices = [129043, 130451, 131860, 133270, 134680, 134681, 134682, 134683, 134686, 134687, 134688, 134689,
           133283, 133284, 131877, 130470, 127655, 129063, 129747, 131156, 132565, 133975, 133976, 135389,
           135390, 133985, 133986, 132580, 129766, 128359]

# Conversione in coordinate
points = index_to_coordinates(indices, width)
circles = cluster_and_enclose_points(points)
plot_clusters_and_circles(points, circles)

print(circles)
