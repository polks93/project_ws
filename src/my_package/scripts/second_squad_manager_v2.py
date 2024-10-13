#!/usr/bin/env python3

import rospy
import numpy as np
import copy
import random
import time

from second_squad_functions import generate_bounding_boxes
from my_package.msg import BoxAssignment
from my_package.srv import GetAgentState, GetAgentStateResponse
from my_package.srv import GetAgentDistance, GetAgentDistanceResponse
from my_package.srv import StartNegotiations, StartNegotiationsResponse

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
        
        self.other_agents = []
        self.state_service_call = {}
        self.distance_service_call = {}
        self.negotiations_service_call = {}
        
        self.other_agents_negotiating = set()
        self.other_agents_choosing = set()
        self.leader = None
        self.negotiation_request = False
        self.my_candidate = None
        self.my_distance = None
        
        for id in range(1, self.n_robot + 1, 1):
            if id is not self.my_id:
                self.other_agents.append(id) 
                   
        # Init agente
        self.box_dict = init_boxes()
        self.current_status = "idle"
        self.next_status = "idle"
        self.prev_status = "idle"
        
        # Init publishers
        self.assigned_pub = rospy.Publisher("/assigned_box", BoxAssignment, queue_size=1, latch=True)
        
        # Init subscribers
        self.assigned_sub = rospy.Subscriber("/assigned_box", BoxAssignment, self.assigned_box_callback)
        
        # Init server
        self.my_state_server = rospy.Service('get_agent_state', GetAgentState, self.handle_agent_state_request)
        self.my_distance_server = rospy.Service('get_agent_distance', GetAgentDistance, self.handle_agent_distance_request)
        self.my_negotiation_server = rospy.Service('start_negotiations', StartNegotiations, self.handle_start_negotiations_request)
        
        # Init clients
        for i in range(len(self.other_agents)):
            prefix = "/robot_" + str(self.other_agents[i])
            agent_state_service = prefix + "/get_agent_state"
            agent_distance_service = prefix + "/get_agent_distance"
            negotitations_service = prefix + "/start_negotiations"
            
            rospy.wait_for_service(agent_state_service)
            rospy.wait_for_service(agent_distance_service)
            rospy.wait_for_service(negotitations_service)
            
            self.state_service_call[self.other_agents[i]] = rospy.ServiceProxy(agent_state_service, GetAgentState)
            self.distance_service_call[self.other_agents[i]] = rospy.ServiceProxy(agent_distance_service, GetAgentDistance)
            self.negotiations_service_call[self.other_agents[i]] = rospy.ServiceProxy(negotitations_service, StartNegotiations)
            
    def handle_agent_state_request(self, req):
        response = GetAgentStateResponse()
        response.agent_state = self.current_status
        self.other_agents_negotiating.add(req.agent_id)
        
        return response
    
    def handle_agent_distance_request(self, req):
        client_distance = req.client_distance
        client_candidate = req.client_candidate
        response = GetAgentDistanceResponse()
        
        if self.my_candidate is not None and self.my_distance is not None:
            if self.my_candidate == client_candidate and self.my_distance < client_distance:
                response.server_closer = True
        else:
            response.server_closer = False
        return response
    
    def handle_start_negotiations_request(self, req):
        response = StartNegotiationsResponse()
        rospy.loginfo(str(self.my_id) + ": Received negotiation request from " + str(req.my_id))
        response.recived = True
        self.next_status = "negotiating"
        self.leader = req.my_id
        self.negotiation_request = True
        
        return response
        
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
            del self.box_dict[msg.id_box]

            if msg.id_agent in self.other_agents_choosing:
                self.other_agents_choosing.remove(msg.id_agent)
            
            if len(self.other_agents_choosing) > 0:
                if self.my_id < min(self.other_agents_choosing):
                    self.leader = self.my_id
                else:
                    self.leader = min(self.other_agents_choosing)
            elif self.current_status == "waiting" or self.current_status == "negotiating":
                self.leader = self.my_id
                
        
    def run(self):
        if self.my_id == 3:
            rospy.sleep(5)
               
        while not rospy.is_shutdown():
            if self.current_status == "idle":
                self.next_status = "choosing_box"
                    
            elif self.current_status == "choosing_box":
                self.handle_choosing_box()
            
            elif self.current_status == "waiting":
                self.handle_waiting()
                if self.negotiation_request:
                    self.next_status = "negotiating"
                    self.negotiation_request = False
            
            elif self.current_status == "negotiating":
                self.handle_negotiation()   
                   
            elif self.current_status == "moving_to_box":
                rospy.loginfo(str(self.my_id) + ": Moving to box " + str(self.my_candidate))
                rospy.sleep(100)
                
            elif self.current_status == "finished":
                rospy.sleep(10)
            
            else:
                rospy.logerr("Invalid status")
            
            self.prev_status = self.current_status
            self.current_status = self.next_status
            self.rate.sleep()
    
    def handle_choosing_box(self):
    
        # Se non ci sono box disponibili, termino
        if not self.box_dict:
            self.next_status = "finished"
            return
        
        # Init other_agents_choosing
        self.other_agents_choosing = set()
        self.other_agents_negotiating = set()
        
        # Imposto un tempo di attesa per avviare la negoziazione
        waiting_time = 3.0
        start_time = time.time()
        
        while time.time() - start_time < waiting_time:
            
            # Continuo a chiedere lo stato degli altri agenti per waiting_time secondi
            for agent in self.other_agents:
                response = self.state_service_call[agent](self.my_id)
                
                # Salvo gli agenti che stanno scegliendo una box
                if response.agent_state == "choosing_box" or response.agent_state == "waiting":
                    self.other_agents_choosing.add(agent)
                
                elif response.agent_state == "negotiating":
                    rospy.loginfo(str(self.my_id) + ": Other agents are negotiating")
                    return
                
            # Esco dal ciclo se tutti gli agenti vogliono negoziare
            if len(self.other_agents_negotiating) == len(self.other_agents):
                break
        
        self.my_candidate = np.random.choice(list(self.box_dict.keys()))
        self.my_distance = np.random.randint(1, 10)
            
        # Se non ci sono agenti che stanno scegliendo una box, scelgo la box che preferisco
        if len(self.other_agents_choosing) == 0:
            rospy.loginfo(str(self.my_id) + ": I'm the only agent choosing a box")
            self.next_status = "moving_to_box"
            return
        
        # Se ci sono agenti altri agenti che stanno scegliendo una box, passo alla negoziazione
        else:
            rospy.loginfo(str(self.my_id)+ ': recived request from : ' + str(self.other_agents_choosing))
            rospy.loginfo(str(self.my_id) + ": Other agents are choosing a box " + str(self.other_agents_choosing))
            self.next_status = "waiting"
            
    def handle_negotiation(self):
        
        rospy.loginfo(str(self.my_id) + ": Negotiating with: " + str(self.other_agents_choosing))
        rospy.loginfo(str(self.my_id) + ": Leader is: " + str(self.leader))
        
        # Il leader chiede a tutti gli agenti se sono più vicini alla box rispetto a lui
        if self.my_id == self.leader:
            for agent in self.other_agents_choosing:
                response = self.distance_service_call[agent](self.my_distance, self.my_candidate)
                if response.server_closer:
                    self.chose_another_box()
                    self.next_state = "negotiating"
                    return
            
            self.next_status = "moving_to_box"
            msg = BoxAssignment()
            msg.id_agent = int(self.my_id)
            msg.id_box = int(self.my_candidate)
            self.assigned_pub.publish(msg)
                

    def handle_waiting(self):
        """
        Gestisce lo stato di attesa dell'agente corrente.
        Questo metodo verifica se l'agente corrente deve attendere altri agenti
        prima di passare allo stato di negoziazione. Se l'ID dell'agente corrente
        è inferiore all'ID minimo degli altri agenti in attesa, invia una richiesta
        di negoziazione a ciascun agente e conta quanti agenti sono pronti. Se tutti
        gli agenti sono pronti, cambia lo stato dell'agente corrente a "negotiating".
        Altrimenti, l'agente corrente attende per un secondo.
        Returns:
            None
        """
        rospy.loginfo(str(self.my_id) + ": Waiting for :" + str(self.other_agents_choosing))
        n_agents = len(self.other_agents_choosing)
        ready_agents = 0
        
        # L'agente con l'ID minore invia le richieste di negoziazione
        if len(self.other_agents_choosing) == 0:
            self.next_status = "moving_to_box"
            return
        
        if self.my_id < min(self.other_agents_choosing):
            for agent in self.other_agents_choosing:
                response = self.negotiations_service_call[agent](self.my_id)
                if response.recived:
                    ready_agents += 1
            
            # Quando ho ricevuto tutte le risposte, passo allo stato di negoziazione, sarò l'ultimo ad essere pronto
            if ready_agents == n_agents:
                self.next_status = "negotiating"
                self.leader = self.my_id
                return
        else:
            start_time = time.time()
            while time.time() - start_time < 5:
                self.next_status = "waiting"
                
                  
if __name__ == '__main__':
    try:
        rospy.init_node('manager_node')
    
        agent = Agent()
        agent.run()
    except rospy.ROSInterruptException:
        pass
    