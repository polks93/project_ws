#!/usr/bin/env python3

import  rospy
import  math
from    std_msgs.msg        import String, Float64
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
    global my_candidate_assigned
    global my_candidate

    waypoint_id         = str(request.waypoint_id)
    waypoint_position   = waypoints[waypoint_id]
    response            = PathLengthResponse()
    response.distance   = distance_calculator(robot_position, waypoint_position)

    response.waiting    = my_candidate_assigned
    if my_candidate is int(waypoint_id):
        response.is_my_candidate = my_candidate_assigned
    else:
        response.is_my_candidate = False

    return response

def callback2(msg):
    x = 1

def waypoint_assignment():

    global enable
    global robot_position
    global waypoints
    global my_candidate_assigned
    global my_candidate

    # Init del nodo
    rospy.init_node("waypoint_assignment")
    rate = rospy.Rate(10)

    #  Acquisizione parametri
    x_0             = rospy.get_param("~x", "0")
    y_0             = rospy.get_param("~y", "0")

    n_robot         = rospy.get_param("/n_robot", "1")      
    my_ns           = rospy.get_namespace()                 
    waypoints       = rospy.get_param("/waypoints_new")
    
    # ID del robot su cui si trova il nodo
    my_id   = int(my_ns[len(my_ns) - 2])

    other_agents = []
    service_call = {}               # Dizionario del tipo 'robot_id' -> service client che contatta quell'agente
    other_waypoint_assigend = {} 
    waypoints_distance = {}

    # La lista other_agents viene riempita con gli id di tutti gli altri agenti
    for id in range(1, n_robot + 1, 1):
        if id is not my_id:
            other_agents.append(id)  
        
    sub = rospy.Subscriber("/chat",             String,     callback,   queue_size=1)
    pub = rospy.Publisher("waypoint_assigned",  WaypointAssigned,       queue_size=3)

    # Init del service server
    my_server = rospy.Service("path_length", PathLength, handle_path_distance_request)
    
    # Init service client e topic sub, uno per ogni altro agente
    for i in range(len(other_agents)):
        prefix = "/robot_" + str(other_agents[i])

        service_name = prefix + "/path_length"
        rospy.wait_for_service(service_name)
        service_call[other_agents[i]] = rospy.ServiceProxy(service_name, PathLength)

        topic_name = prefix + "/waypoint_assigned"
        other_waypoint_assigend[other_agents[i]] = rospy.Subscriber(topic_name, WaypointAssigned, callback2, queue_size=3)

    # Init messaggio da pubblicare
    result_msg  = WaypointAssigned()
    result_msg.robot_id = my_id

    # Valori iniziali variabli ancora non assegnate
    robot_position = [x_0, y_0]
    enable = False
    my_candidate_assigned = False
    my_candidate = 0

    prev_state = 'CALCULATE DISTANCE'
    curr_state = 'CALCULATE DISTANCE'
    next_state = 'CALCULATE DISTANCE'

    while not rospy.is_shutdown():
        if enable is True:

            if curr_state == 'CALCULATE DISTANCE':

                
                # Ciclo for che calcola la distanza da tutti gli waypoint
                for i in range(len(waypoints)):
                    # list() fornisce una lista di tutte le chiavi presenti nel dizionario 'waypoint'
                    id_waypoint = list(waypoints)[i]

                    # Dizionario: ID waypoint -> distanza dal waypoint
                    waypoints_distance[id_waypoint] = distance_calculator(robot_position, waypoints[id_waypoint])
                
                # Lista ordinata del dizionario creato sopra del tipo [('id', distanza), ...]
                waypoints_orderd = sorted(waypoints_distance.items(), key=lambda item:item[1])

                next_state = 'DECISION MAKING'
                k = 0
                agent_waiting = 0

            elif curr_state == 'DECISION MAKING':
                # Id del waypoint con la distanza minore 
                my_candidate   = int(waypoints_orderd[k][0])
                my_distance    = waypoints_orderd[k][1]

                for i in range(len(other_agents)):

                    if k == n_robot - 1:
                        rospy.loginfo("waiting....")

                        next_state = 'WAITING'
                        break

                    already_assigned = service_call[other_agents[i]](my_candidate).is_my_candidate

                    if already_assigned is True:
                        k += 1
                        break

                    else:
                        other_distance = service_call[other_agents[i]](my_candidate).distance
                    
                        if my_distance > other_distance or (my_distance == other_distance and my_id > other_agents[i]):
                            k += 1
                            break

                    next_state = 'WAITING'

            elif curr_state == 'WAITING':
                my_candidate_assigned = True
                result_msg.waypoint_id = my_candidate

                for i in range(len(other_agents)):
                    agent_waiting = agent_waiting + service_call[other_agents[i]](1).waiting

                if agent_waiting == (n_robot - 1):
                    next_state = 'PUBLISHING'

            elif curr_state == 'PUBLISHING':
                if curr_state != prev_state:
                    pub.publish(result_msg)

            prev_state = curr_state
            curr_state = next_state

        rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_assignment()
    except rospy.ROSInterruptException:
        pass