#!/usr/bin/env python3

from    __future__          import print_function

import  rospy
import  math
from    std_msgs.msg        import String, Float64, Int8
from    my_package.srv      import PathLength, PathLengthResponse
from    my_package.msg      import WaypointAssigned

#  Callback che fa partire l'esecuzione del nodo
def callback(msg):
    
    global enable

    enable = True

#  Funzione per calcoalre la distanza euclidea tra 2 punti
def distance_calculator(P1, P2):

    x1 = P1[0]
    y1 = P1[1]

    x2 = P2[0]
    y2 = P2[1]

    distance = math.pow(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2), 0.5)

    return distance

# Service server presente in ogni nodo:
# Quando viene ricevuta una richiesta contenente l'id di un waypoint
# restituisce la distanza del robot da quel punto
def handle_path_distance_request(request):

    global waypoints
    global robot_position

    waypoint_id         = str(request.waypoint_id)
    waypoint_position   = waypoints[waypoint_id]
    response            = PathLengthResponse()
    response.distance = distance_calculator(robot_position, waypoint_position)
    return response

def remove_waypoint(msg):

    global other_agent_step_completed
    global waypoints
    global waypoints_orderd
    global waypoints_distance
    global other_agents_to_check
    global my_id
    global waypoint_recived

    robot_id = msg.robot_id
    waypoint_id = str(msg.waypoint_id)

    del waypoints[waypoint_id]
    del waypoints_distance[waypoint_id]

    waypoints_orderd = sorted(waypoints_distance.items(), key=lambda item:item[1])

    waypoint_recived = waypoint_recived + 1
    other_agents_to_check[robot_id] = False
    other_agent_step_completed = True


def waypoint_assignment():

    global enable
    global robot_position
    global waypoints
    global waypoints_orderd
    global waypoints_distance
    global my_id
    global other_agents_to_check
    global other_agent_step_completed
    global waypoint_recived

    # Init del nodo
    rospy.init_node("waypoint_assignment")
    rate = rospy.Rate(10)

    #  Acquisizione parametri
    x_0             = rospy.get_param("~x", "0")
    y_0             = rospy.get_param("~y", "0")
    # Y_0             = rospy.get_param("~Y", "0")

    n_robot         = rospy.get_param("/n_robot", "1")      
    my_ns           = rospy.get_namespace()                 
    waypoints       = rospy.get_param("/waypoints_new")
    
    # ID del robot su cui si trova il nodo
    my_id   = int(my_ns[len(my_ns) - 2])
    
    # Init varabili
    other_agents = []                   # Init lista degli ID degli altri robot
    other_agents_to_check = {}
    my_waypoints = []               # Lista waypoints assegnati a me
    waypoints_distance = {}         # Dizionario del tipo 'waypoint_id' -> distanza dal waypoint
    service_call = {}               # Dizionario del tipo 'robot_id' -> service client che contatta quell'agente
    other_waypoint_assigend = {}    # Dizionario del tipo 'robot_id' -> subscriber al topic in cui gli altri agenti 
                                    # pubblicano i loro waypoint

    # La lista other_agents viene riempita con gli id di tutti gli altri agenti
    for id in range(1, n_robot + 1, 1):
        if id is not my_id:
            other_agents.append(id)  
            other_agents_to_check[id] = True

    # Topic per debug
    sub = rospy.Subscriber("/chat",             String,     callback, queue_size=1)
    pub_test = rospy.Publisher("test",          Float64,    queue_size=2)
    pub = rospy.Publisher("waypoint_assigned",  WaypointAssigned,       queue_size=3)

    # Init del service server
    my_server = rospy.Service("path_length", PathLength, handle_path_distance_request)
     
    # Init service client, uno per ogni altro agente
    for i in range(len(other_agents)):
        prefix = "/robot_" + str(other_agents[i])

        service_name = prefix + "/path_length"
        rospy.wait_for_service(service_name)
        service_call[other_agents[i]] = rospy.ServiceProxy(service_name, PathLength)

        topic_name = prefix + "/waypoint_assigned"
        other_waypoint_assigend[other_agents[i]] = rospy.Subscriber(topic_name, WaypointAssigned, remove_waypoint, queue_size=3)

    # Init messaggi da pubblicare
    test_msg    = Float64()
    result_msg  = WaypointAssigned()
    result_msg.robot_id = my_id
    
    # Valori iniziali variabli ancora non assegnate
    robot_position = [x_0, y_0]
    enable = False
    other_agent_step_completed = False

    waypoint_recived = 0

    prev_state = 'CALCULATE DISTANCE'
    curr_state = 'CALCULATE DISTANCE'
    next_state = 'CALCULATE DISTANCE'

    while not rospy.is_shutdown():

        if enable is True:

            if curr_state == 'CALCULATE DISTANCE':
                n_waypoints = len(waypoints)    

                # Ciclo for che calcola la distanza da tutti gli waypoint
                for i in range(n_waypoints):
                    # list() fornisce una lista di tutte le chiavi presenti nel dizionario 'waypoint'
                    id_waypoint = list(waypoints)[i]

                    # Dizionario: ID waypoint -> distanza dal waypoint
                    waypoints_distance[id_waypoint] = distance_calculator(robot_position, waypoints[id_waypoint])
                
                # Lista ordinata del dizionario creato sopra del tipo [['id', distanza], ...]
                waypoints_orderd = sorted(waypoints_distance.items(), key=lambda item:item[1])

                # Id del waypoint con la distanza minore 
                my_candidate   = int(waypoints_orderd[0][0])
                my_distance    = waypoints_orderd[0][1]

                next_state = 'DECISION MAKING'

            elif curr_state == 'DECISION MAKING':
                if sum(other_agents_to_check.values()) > 0:
                
                    # Chiamata al server di tutti gli altri agenti per ottenere la loro distanza da
                    # my_candidate
                    for i in range(len(other_agents)):
                        if other_agents_to_check[other_agents[i]] is True:
                            rospy.loginfo("robot %s calling service to %s", my_id, other_agents[i])
                            # Richiedo la distanza a other_agents[i]
                            other_dist = service_call[other_agents[i]](my_candidate).distance

                            if other_dist < my_distance:
                                next_state = 'WAITING OTHER AGENTS'
                                break
                            if other_dist == my_distance and my_id > other_agents[i]:
                                next_state = 'WAITING OTHER AGENTS'
                                break
                            
                            next_state = 'WAYPOINT ASSIGNED'
                else:
                    next_state = 'WAYPOINT ASSIGNED'

            elif curr_state == 'WAITING OTHER AGENTS':

                if other_agent_step_completed is True:
                    my_candidate =  int(waypoints_orderd[0][0])
                    my_distance =   waypoints_orderd[0][1]

                    other_agent_step_completed = False
                    next_state = 'DECISION MAKING'

            elif curr_state == 'WAYPOINT ASSIGNED':
                
                if curr_state is not prev_state: 
                    result_msg.waypoint_id = my_candidate
                    pub.publish(result_msg)

                if waypoint_recived == (n_robot - 1):
                    robot_position = waypoints[str(my_candidate)]
                    waypoint_recived = 0
                    del waypoints[str(my_candidate)]
                    for id in range(len(other_agents)):
                        other_agents_to_check[other_agents[id]] = True
                    next_state = 'CALCULATE DISTANCE'   
                    enable = False

      
            prev_state = curr_state
            curr_state = next_state

        rate.sleep()
if __name__ == '__main__':
    try:
        waypoint_assignment()
    except rospy.ROSInterruptException:
        pass
