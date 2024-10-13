#!/usr/bin/env python3

import rospy
import random
from my_package.srv import NegotiateArea
from second_squad_functions import generate_bounding_boxes

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
   
class AgentNode:
    def __init__(self):
        
        self.n_robot = rospy.get_param("/n_robot", "1")
        self.my_ns   = rospy.get_namespace()
        self.my_id   = int(self.my_ns[len(self.my_ns) - 2])
        self.assigned_area = None
        self.request_pending = True
        self.rate = rospy.Rate(1)
        
        rospy.wait_for_service('/negotiate_area')
        self.negotiate_service = rospy.ServiceProxy('/negotiate_area', NegotiateArea)
        
        self.available_areas = init_boxes()
        rospy.loginfo("Agent %d initialized", self.my_id)
        
    def negotiate_area(self):
        
        rospy.loginfo("Agent %d inizia la negoziazione", self.my_id)
        while not rospy.is_shutdown() and self.available_areas:
            area_id = random.choice(list(self.available_areas.keys()))
            distance = random.uniform(1.0, 10.0)
            
            rospy.loginfo("Agent %d negotiating area %d", self.my_id, area_id)
                    
            try:
                response = self.negotiate_service(self.my_id, area_id, distance)
                if response.success:
                    rospy.loginfo(f"Agente {self.my_id}: {response.message}")
                    # Attendi l'assegnazione definitiva
                    self.wait_for_assignment()
                    if self.assigned_area is not None:
                        break  # Area assegnata, esci dal ciclo
                else:
                    rospy.logwarn(f"Agente {self.my_id}: {response.message}")
                    if area_id in self.available_areas:
                        del self.available_areas[area_id]
                    
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                rospy.sleep(1.0)
                
    def wait_for_assignment(self):
        while not rospy.is_shutdown():
            try:
                response = self.negotiate_service(self.my_id, -1, 0.0)
                if response.assigned_area_id != -1:
                    if response.success:
                        rospy.loginfo(f"Agente {self.my_id}: {response.message}")
                        self.assigned_area = response.assigned_area_id
                    else: 
                        rospy.logwarn(f"Agente {self.my_id}: {response.message}")
                    break
                elif response.assigned_area_id == -1:
                    rospy.loginfo(f"Agente {self.my_id}: In attesa di assegnazione")
                    
                elif response.assigned_area_id == -2:
                    rospy.logwarn(f"Agente {self.my_id}: Assegnazione rifiutata, un altro agente è più vicino")
                    if self.assigned_area in self.available_areas:
                        del self.available_areas[self.assigned_area]
                    break
                
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            self.rate.sleep()
            
    def run(self):
        while not rospy.is_shutdown():
            initial_delay = random.uniform(0.0, 1.0)
            rospy.sleep(initial_delay)
            
            
            self.negotiate_area()
            
            if self.assigned_area is not None:
                if self.assigned_area in self.available_areas:
                    del self.available_areas[self.assigned_area]
                rospy.loginfo(f"Agente {self.my_id} Inizio esplorazione area {self.assigned_area}")
                exploration_time = random.uniform(5,10)
                rospy.sleep(exploration_time)
                rospy.loginfo(f"Agente {self.my_id} Esplorazione area {self.assigned_area} completata")
                self.assigned_area = None
                
            else:
                rospy.logwarn(f"Agente {self.my_id} Nessuna area assegnata")
        
            self.rate.sleep()
            
            if len(self.available_areas) == 0:
                break
        rospy.loginfo(f"Agente {self.my_id} Esplorazione completata")
        rospy.signal_shutdown("Esplorazione completata")
        
if __name__ == '__main__':
    try: 
        rospy.init_node('agent_node')
        agent = AgentNode()  
        agent.run()
        
    except rospy.ROSInterruptException:
        pass
    