#!/usr/bin/env python3

import rospy
import numpy as np
import copy
from second_squad_functions import generate_bounding_boxes
from my_package.msg import AgentState
from my_package.msg import BoxAssignment
from my_package.msg import ProposedBox

class MonitorAgents:
    """
    La classe MonitorAgents monitora lo stato di un gruppo di agenti.
    Attributi:
    ----------
    agents_states : dict
        Un dizionario che mappa l'ID dell'agente al suo stato attuale.
    n_agents : int
        Il numero totale di agenti nel gruppo.
    my_id : int
        L'ID dell'agente corrente.
    subscriber : rospy.Subscriber
        Un sottoscrittore ROS per il topic "/agents_states".
    Metodi:
    -------
    __init__(n_agents, my_id)
        Inizializza la classe con il numero di agenti e l'ID dell'agente corrente.
    callback_agent_state(msg)
        Callback per aggiornare lo stato degli agenti quando viene ricevuto un messaggio.
    get_agents_states()
        Restituisce una copia dello stato attuale degli agenti.
    print_status()
        Stampa lo stato attuale degli agenti nei log ROS.
    """

    def __init__(self, n_agents, my_id):
        self.agents_states = {}
        self.n_agents = n_agents
        self.my_id = my_id
        for i in range(1,n_agents+1):
            self.agents_states[i] = "idle"
        self.subscriber = rospy.Subscriber("/agents_states", AgentState, self.callback_agent_state)
        
    def callback_agent_state(self, msg):
        """
        Callback per aggiornare lo stato degli agenti.
        Questo metodo viene chiamato quando viene ricevuto un messaggio con lo stato di un agente.
        Aggiorna lo stato dell'agente nella variabile `agents_states` e stampa lo stato aggiornato.
        Args:
            msg: Il messaggio contenente l'ID e lo stato dell'agente.
        """
        self.agents_states[msg.id_agent] = msg.status
        self.print_status()
        
    def get_agents_states(self):
        """
        Restituisce una copia profonda degli stati degli agenti.
        Returns:
            dict: Una copia del dizionario degli stati degli agenti.
        """
        return copy.deepcopy(self.agents_states)
    
    def print_status(self):
        """
        Stampa lo stato corrente degli agenti.
        Utilizza rospy.loginfo per registrare le informazioni sugli stati degli agenti.
        Returns:
            None
        """
        rospy.loginfo(str(self.my_id) + ": " + str(self.agents_states))
        
def init_boxes():
    """ 
    Calcolo i riquadri da esplorare a partire da circles disponibile come parametro globale 
    
    Returns:
        dict: Un dizionario che rappresenta i riquadri da esplorare con chiavi 'box' e 'center'.
                'box' è una tupla contenente le coordinate del riquadro di delimitazione (x_min, y_min, x_max, y_max).
                'center' è una tupla contenente le coordinate del centro del riquadro (Cx, Cy).
    """
    workspace = rospy.get_param("/workspace")
    circles = rospy.get_param("/circles")
    merged_boxes = generate_bounding_boxes(circles, workspace)
    
    return merged_boxes

def assigned_box_callback(msg):
    global assigned_boxes
    global boxes
    
    assigned_boxes.add(msg.id_box)
    del boxes[msg.id_box]

def proposed_box_callback(msg):
    global recived_proposals
    global my_id
    
    if msg.id_agent != my_id:
        recived_proposals.append(msg.id_box)
         
def second_squad_manager():
    global assigned_boxes
    global boxes
    global recived_proposals
    global my_id
    
    assigned_boxes = set()
    recived_proposals = []
    
    rospy.init_node('manager_node')
    rate = rospy.Rate(1)
    
    # Import parametri
    x_0     = rospy.get_param("~x", "0")
    y_0     = rospy.get_param("~y", "0")
    n_robot = rospy.get_param("/n_robot", "1")
    my_ns   = rospy.get_namespace()
    my_id   = int(my_ns[len(my_ns) - 2])
    
    # Dizionario box da esplorare
    boxes = init_boxes()
    
    # Init publisers
    state_pub       = rospy.Publisher("/agents_states", AgentState, queue_size=1, latch=True)
    box_pub         = rospy.Publisher("/box_assignment", BoxAssignment, queue_size=1, latch=True)
    proposed_pub    = rospy.Publisher("/proposed_box", ProposedBox, queue_size=1, latch=True)
    
    # Init subscribers
    box_sub         = rospy.Subscriber("/box_assignment", BoxAssignment, assigned_box_callback)
    proposed_sub    = rospy.Subscriber("/proposed_box", ProposedBox, proposed_box_callback)

    # Creo l'oggetto per monitorare gli stati di tutti gli agenti
    monitor = MonitorAgents(n_robot, my_id)
    


    status = "chosing_box"
    
    rospy.sleep(1)

    while not rospy.is_shutdown():
        state_pub.publish(AgentState(id_agent=my_id, status=status))

        if status == "chosing_box":
            rospy.loginfo(my_ns + ": Sono in stato chosing_box")
            my_candidate = np.random.choice(list(boxes.keys()))
            my_distance = np.random.rand() * 10
            next_status = "moving_to_box"
            rospy.sleep(1)

        elif status == "moving_to_box":
            rospy.loginfo(my_ns + ": Sono in stato moving_to_box")
            next_status = "exploring_box"
            rospy.sleep(1)

        elif status == "exploring_box":
            rospy.loginfo(my_ns + ": Sono in stato exploring_box")
            rospy.sleep(1)

            next_status = "finished"
        elif status == "finished":
            rospy.loginfo(my_ns + ": Sono in stato finished")
            # rospy.signal_shutdown("Ho finito")
            rospy.sleep(1)
       
        status = next_status
        rate.sleep()

if __name__ == '__main__':
    try:
        second_squad_manager()
    except rospy.ROSInterruptException:
        pass