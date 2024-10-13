#!/usr/bin/env python3

import rospy
import random
import numpy as np
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv        import GetPlan
from move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal

from my_package.srv import NegotiateArea
from second_squad_functions import generate_bounding_boxes

GREEN = "\033[92m"
RESET = "\033[0m"

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
def distance(p1, p2):
    """
    Calcola la distanza euclidea tra due punti 2D.
    Args:
        p1 (tuple): Coordinata del primo punto (x1, y1).
        p2 (tuple): Coordinata del secondo punto (x2, y2).
    Returns:
        float: Distanza euclidea tra i due punti.
    """
    
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
   
class AgentNode:
    def __init__(self):
        self.x_0 = rospy.get_param("~x", "0")
        self.y_0 = rospy.get_param("~y", "0")
        self.n_robot = rospy.get_param("/n_robot", "1")
        self.my_ns   = rospy.get_namespace()
        self.my_id   = int(self.my_ns[len(self.my_ns) - 2])
        self.assigned_area = None
        self.request_pending = True
        self.rate = rospy.Rate(1)
        self.available_areas = init_boxes()
        self.current_pose = None
        self.safe_distance = 0.25
        self.current_goal = None
        
        # Init subscriber
        self.amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)

        # Init service client
        rospy.wait_for_service('/negotiate_area')
        self.negotiate_service = rospy.ServiceProxy('/negotiate_area', NegotiateArea)
        
        rospy.wait_for_service('move_base/make_plan')
        self.get_plan_service = rospy.ServiceProxy('move_base/make_plan', GetPlan)
        
        self.goal_service = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.goal_service.wait_for_server()
        
        rospy.loginfo("Agent %d initialized", self.my_id)

    def amcl_callback(self, msg):
        """ 
        Callback per topic disponibile su /amcl_pose
        Salva la posa corrente dell'agente
        """
        self.current_pose = msg.pose.pose
             
    def negotiate_area(self):
        
        rospy.loginfo(f"{GREEN}Agent %d inizia la negoziazione{RESET}", self.my_id)
        while not rospy.is_shutdown() and self.available_areas:
            
            area_id, distance = self.get_closer_area()
            
            if area_id is None:
                rospy.loginfo(f"Agente {self.my_id}: Non posso calcolara l'area più vicina")
                break
            
            rospy.loginfo("Agent %d negotiating area %d", self.my_id, area_id)
                    
            try:
                response = self.negotiate_service(self.my_id, area_id, distance)
                if response.success:
                    rospy.loginfo(f"Agente {self.my_id}: {response.message}")
                    # Attendi l'assegnazione definitiva
                    self.wait_for_assignment(area_id)
                    if self.assigned_area is not None:
                        break  # Area assegnata, esci dal ciclo
                else:
                    rospy.logwarn(f"Agente {self.my_id}: {response.message}")
                    if area_id in self.available_areas:
                        del self.available_areas[area_id]
                    
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                rospy.sleep(1.0)
 
    def get_closer_area(self):
        """
        Calcola l'area più vicina all'agente interrogando move_base/make_plan
        
        Returns:
            int: ID dell'area più vicina
            float: Distanza dall'area più vicina
        """
        
        # Se non ho ancora una posa esco
        if self.current_pose is None:
            rospy.logwarn(f"Agente {self.my_id}: AMCL non disponibile")
            return None, None
        
        # Creo la posa di partenza
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x = self.current_pose.position.x
        start.pose.position.y = self.current_pose.position.y
        start.pose.orientation.z = self.current_pose.orientation.z
        start.pose.orientation.w = self.current_pose.orientation.w

        # Caclolo la distanza tra l'agente e le aree disponibili
        min_distance = None
        for id, area in self.available_areas.items():
            
            # Genero il goal
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = area['center'][0]
            goal.pose.position.y = area['center'][1]
            goal.pose.orientation.w = 1.0
            
            tolerance = 0.25
            
            # Invio la richiesta di percorso
            response = self.get_plan_service(start, goal, tolerance)
            path = response.plan.poses
            
            # Calcolo la lunghezza del percorso e il goal da raggiungere
            path_lenght, goal = self.truncate_path_at_rectangle(path, area['box'])
            
            rospy.loginfo(f"Agente {self.my_id}: Area {id} distanza {path_lenght:.2f}")
            # Salvo la distanza minima
            if min_distance is None or path_lenght < min_distance:
                min_distance = path_lenght
                area_id = id
                self.current_goal = goal
        
        rospy.loginfo(f"Agente {self.my_id}: Area più vicina {area_id} distanza {min_distance:.2f}")      
        # area_id = random.choice(list(self.available_areas.keys()))
        # distance = random.uniform(1.0, 10.0)
        return area_id, min_distance
    
    def truncate_path_at_rectangle(self, path, rect):
        x_min, y_min, x_max, y_max = rect
        x_min += self.safe_distance
        y_min += self.safe_distance
        x_max -= self.safe_distance
        y_max -= self.safe_distance
        
        x0, y0 = path[0].pose.position.x, path[0].pose.position.y
        P0 = (x0, y0)
        
        path_lenght = 0
        
        for pose_stamped in path:
            # Ottieni le coordinate x e y del punto nel path
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            P = (x, y)

            path_lenght += distance(P0, P)

            P0 = P
            # Verifica se il punto si trova all'interno dei limiti del rettangolo
            if x_min <= x <= x_max and y_min <= y <= y_max:
                # Se il punto è all'interno del rettangolo, termina l'iterazione
                goal = pose_stamped
                break
            
        return path_lenght, goal
                 
    def wait_for_assignment(self, area_id):
        
        while not rospy.is_shutdown():
            try:
                response = self.negotiate_service(self.my_id, -1, 0.0)
                
                if response.assigned_area_id != -1 and response.assigned_area_id != -2:
                    
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
                    # Rimuovo l'area assegnata da quelle disponibili
                    if area_id in self.available_areas:
                        del self.available_areas[area_id]
                    break
                
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            self.rate.sleep()
    
    def move_to_area(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.current_goal.pose.position.x
        goal.target_pose.pose.position.y = self.current_goal.pose.position.y
        goal.target_pose.pose.orientation.z = self.current_goal.pose.orientation.z
        goal.target_pose.pose.orientation.w = self.current_goal.pose.orientation.w
        self.goal_service.send_goal(goal)
        self.goal_service.wait_for_result()
        rospy.loginfo(f"Agente {self.my_id} Arrivato all'area {self.assigned_area}")
                
    def explore_area(self):
        explore_time = random.uniform(5.0, 10.0)
        # rospy.sleep(100)

    def homing(self):
        rospy.loginfo(f"Agente {self.my_id} Ritorno al punto di partenza")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.x_0
        goal.target_pose.pose.position.y = self.y_0
        goal.target_pose.pose.orientation.w = 1.0
        self.goal_service.send_goal(goal)
        self.goal_service.wait_for_result()
        rospy.loginfo(f"Agente {self.my_id} Ritorno al punto di partenza completato")           
    
    def run(self):
        while not rospy.is_shutdown():
            initial_delay = random.uniform(0.0, 1.0)
            rospy.sleep(initial_delay)
            
            
            self.negotiate_area()
            
            if self.assigned_area is not None:
                # RImuovo area assegnata da quelle disponibili
                if self.assigned_area in self.available_areas:
                    del self.available_areas[self.assigned_area]
                    
                rospy.loginfo(f"Agente {self.my_id} Inizia a muoversi verso l'area {self.assigned_area}")
                self.move_to_area()
                
                rospy.loginfo(f"Agente {self.my_id} Inizia l'esplorazione dell'area {self.assigned_area}")
                self.explore_area() 
                rospy.loginfo(f"Agente {self.my_id} Esplorazione area {self.assigned_area} completata")
                
                self.assigned_area = None
                
            else:
                rospy.logwarn(f"Agente {self.my_id} Nessuna area assegnata")
        
            self.rate.sleep()
            
            # Quando le aree sono finite torno al punto di partenza
            if len(self.available_areas) == 0:
                self.homing()
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
    