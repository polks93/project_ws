#!/usr/bin/env python3

import rospy
import numpy as np
import copy
import random

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
        
class Agent():
    def __init__(self):
        # Init ROS
        self.rate = rospy.Rate(1)
        
        # Import parametri
        self.x_0     = rospy.get_param("~x", "0")
        self.y_0     = rospy.get_param("~y", "0")
        self.n_robot = rospy.get_param("/n_robot", "1")
        self.my_ns   = rospy.get_namespace()
        self.my_id   = int(self.my_ns[len(self.my_ns) - 2])
                
        # Init agente
        self.box_dict = init_boxes()
        self.current_status = "idle"
        self.next_status = "idle"
        self.prev_status = "idle"
        
        self.assigned_box = set()
        self.proposed_recived = []
        self.my_box = None
        
        # Init publishers
        self.state_pub = rospy.Publisher("/agents_states", AgentState, queue_size=1, latch=True)
        self.assigned_pub = rospy.Publisher("/assigned_box", BoxAssignment, queue_size=1, latch=True)
        self.proposed_pub = rospy.Publisher("/proposed_box", ProposedBox, queue_size=1, latch=True)
        
        # Init subscribers
        self.assigned_sub = rospy.Subscriber("/assigned_box", BoxAssignment, self.assigned_box_callback)
        self.proposed_sub = rospy.Subscriber("/proposed_box", ProposedBox, self.proposed_box_callback)
        
        # Creo l'oggetto MonitorAgents per monitorare lo stato degli agenti
        self.Monitor = MonitorAgents(self.n_robot, self.my_id)
        
        # Sleep per permettere la registrazione dei publishers e subscribers
        rospy.sleep(0.5)
        
        
    def assigned_box_callback(self, msg):
        """
        Callback per gestire l'assegnazione di un'area a un agente.

        Questo metodo viene chiamato quando viene ricevuto un messaggio che indica
        l'assegnazione di un'area a un agente. Se l'agente nel messaggio non è
        l'agente corrente, l'area viene aggiunta all'insieme delle aree assegnate
        e rimossa dal dizionario delle aree disponibili.

        Args:
            msg (Message): Il messaggio contenente l'ID dell'agente e l'ID della scatola.
        """
        if msg.id_agent != self.my_id:
            self.assigned_box.add(msg.id_box)
            del self.box_dict[msg.id_box]

    def proposed_box_callback(self, msg):
        """
        Callback per gestire la proposta di un'area da parte di un agente.

        Questo metodo viene chiamato quando viene ricevuto un messaggio che indica
        la proposta di un'area da parte di un agente. Se l'agente nel messaggio non è
        l'agente corrente, l'area viene aggiunta alla lista delle aree proposte.

        Args:
            msg (Message): Il messaggio contenente l'ID dell'agente e l'ID dell'area.
        """
        if msg.id_agent != self.my_id:
            self.proposed_recived.append(msg)
        
    def run(self):
        while not rospy.is_shutdown():
            self.state_pub.publish(AgentState(id_agent=self.my_id, status=self.current_status))
            if self.current_status == "idle":
                self.next_status = "choosing_box"
                
            elif self.current_status == "choosing_box":
                self.handle_choosing_box()
                
            elif self.current_status == "moving_to_box":
                # rospy.loginfo(str(self.my_id) + ": Moving to box " + str(self.my_box))
                rospy.sleep(10)
            elif self.current_status == "waiting":
                self.next_status = "finished"
                
            elif self.current_status == "finished":
                if self.current_status != self.prev_status:
                    rospy.sleep(10)
            
            self.prev_status = self.current_status
            self.current_status = self.next_status
            self.rate.sleep()
    
    def handle_choosing_box(self):
        
        if not self.box_dict:
            self.next_status = "finished"
            return
        
        waiting_time = random.uniform(1, 3)
        rospy.sleep(waiting_time)
        agent_states = self.Monitor.get_agents_states()
        
        if 'negoziating' in agent_states.values():
            rospy.loginfo(str(self.my_id) + ": Waiting for other agents to finish negoziating")
            self.next_status = self.current_status
            return
        
        # Controllo se ci sono altri agenti che stanno scegliendo un box
        other_agents_choosing = [id for id, status in agent_states.items() if status == "choosing_box" and id != self.my_id]
        rospy.loginfo(str(self.my_id) + ": Other agents choosing: " + str(other_agents_choosing))
        self.my_candidate = random.choice(list(self.box_dict.keys()))
        self.my_distance = np.random.rand() * 10
        rospy.loginfo(str(self.my_id) + ": Choosing box " + str(self.my_candidate) + " with distance " + str(self.my_distance))
        
        if len(other_agents_choosing) > 0:
            # Passo allo stato temporaneo di negoziazione
            self.current_status = "negoziating"
            rospy.loginfo(str(self.my_id) + ": Negozio con altri agenti")
            self.state_pub.publish(AgentState(id_agent=self.my_id, status=self.current_status))
            self.negoziating()
        else:
            rospy.loginfo(str(self.my_id) + ": Nessun agente sta negoziando")
            self.assigned_box.add(self.my_candidate)
            del self.box_dict[self.my_candidate]
            self.assigned_pub.publish(BoxAssignment(id_agent=self.my_id, id_box=self.my_candidate))
            self.next_status = "moving_to_box"
            self.my_box = self.my_candidate
            rospy.loginfo(str(self.my_id) + ": Moving to box " + str(self.my_box))
            
    def negoziating(self):
        my_candidate = self.my_candidate
        my_distance = self.my_distance
        
        propose_msg = ProposedBox(id_agent=self.my_id, id_box=my_candidate, distance=my_distance)
        
        self.proposed_pub.publish(propose_msg)
        
        # Aspetto che tutti gli agenti abbiano proposto un box
        rospy.sleep(1)
        
        same_box_proposals = [msg for msg in self.proposed_recived if msg.id_box == my_candidate]
        if not same_box_proposals:
            self.assigned_box.add(my_candidate)
            del self.box_dict[my_candidate]
            self.assigned_pub.publish(BoxAssignment(id_agent=self.my_id, id_box=my_candidate))
            self.next_status = "moving_to_box"
            self.my_box = my_candidate
            rospy.loginfo(str(self.my_id) + ": Moving to box " + str(self.my_box))
        
            return
        
        min_distance_other_agents = min(same_box_proposals, key=lambda x: x.distance)
        
        if min_distance_other_agents.distance > my_distance:
            self.assigned_box.add(my_candidate)
            del self.box_dict[my_candidate]
            self.assigned_pub.publish(BoxAssignment(id_agent=self.my_id, id_box=my_candidate))
            self.next_status = "moving_to_box"
            self.my_box = my_candidate
            rospy.loginfo(str(self.my_id) + ": Moving to box " + str(self.my_box))
        else:
            self.next_status = "choosing_box"
            rospy.loginfo(str(self.my_id) + ": Choosing another box, mine is closer to other agents")
        self.proposed_recived = []
            
if __name__ == '__main__':
    try:
        rospy.init_node('manager_node')
    
        agent = Agent()
        agent.run()
    except rospy.ROSInterruptException:
        pass
    