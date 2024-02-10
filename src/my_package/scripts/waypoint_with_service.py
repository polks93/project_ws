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
    global my_candidate
    global ready
    
    response = PathLengthResponse()

    if request.waypoint_id == 0:
        response.ready = ready

    else:
        waypoint_id = str(request.waypoint_id)

        if my_candidate is int(waypoint_id) and ready == True:
            response.is_my_candidate = True

        else:
            waypoint_position   = waypoints[waypoint_id]
            response.distance   = distance_calculator(robot_position, waypoint_position)
            response.is_my_candidate = False

    return response

def waypoint_assigned_callback(msg):
    global callback_recived

    robot_id = msg.robot_id
    waypoint_id = str(msg.waypoint_id)
    callback_recived[robot_id] = [True, waypoint_id]

def waypoint_assignment():
    global my_id
    global enable
    global robot_position
    global waypoints
    global my_candidate
    global curr_state
    global waiting
    global ready
    global callback_recived

    # Init del nodo
    rospy.init_node("waypoint_assignment")
    rate = rospy.Rate(30)

    #  Acquisizione parametri
    x_0             = rospy.get_param("~x", "0")
    y_0             = rospy.get_param("~y", "0")

    n_robot         = rospy.get_param("/n_robot", "1")      
    my_ns           = rospy.get_namespace()                 
    waypoints       = rospy.get_param("/waypoints_new")
    
    # ID del robot su cui si trova il nodo
    my_id   = int(my_ns[len(my_ns) - 2])

    waypoints_avaible = waypoints
    my_waypoints = []
    other_agents = []
    service_call = {}               # Dizionario del tipo 'robot_id' -> service client che contatta quell'agente
    other_waypoint_assigend = {} 
    other_distance_dic = {}
    waypoints_distance = {}
    callback_recived = {}

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
        other_waypoint_assigend[other_agents[i]] = rospy.Subscriber(topic_name, WaypointAssigned, waypoint_assigned_callback, queue_size=3)
        
        #  Init dizionario globale callback
        callback_recived[other_agents[i]] = [False, '']

    # Init messaggio da pubblicare
    result_msg  = WaypointAssigned()
    result_msg.robot_id = my_id

    # Valori iniziali variabli ancora non assegnate
    robot_position = [x_0, y_0]
    enable =                False
    my_candidate_assigned = False
    waiting =               False
    ready =                 False
    go_next =               False
    my_candidate = 0
    agent_waiting = 0

    prev_state = 'CALCULATE DISTANCE'
    curr_state = 'CALCULATE DISTANCE'
    next_state = 'CALCULATE DISTANCE'

    while not rospy.is_shutdown():

        if enable is True:

            if curr_state == 'CALCULATE DISTANCE':

                waypoints_distance = {}
                waypoints_ordered = []

                waiting = False

                # Ciclo for che calcola la distanza da tutti gli waypoint
                for i in range(len(waypoints_avaible)):
                    # list() fornisce una lista di tutte le chiavi presenti nel dizionario 'waypoint'
                    id_waypoint = list(waypoints_avaible)[i]

                    # Dizionario: ID waypoint -> distanza dal waypoint
                    waypoints_distance[id_waypoint] = distance_calculator(robot_position, waypoints_avaible[id_waypoint])
                
                # Lista ordinata del dizionario creato sopra del tipo [('id', distanza), ...]
                waypoints_ordered = sorted(waypoints_distance.items(), key=lambda item:item[1])

                next_state = 'DECISION MAKING'
                k = 0

            elif curr_state == 'DECISION MAKING':

                ready = False
                go_next = False
                my_candidate_assigned = False
                other_distance_dic = {}

                # Id del waypoint con la distanza minore 
                my_candidate   = int(waypoints_ordered[k][0])
                my_distance    = waypoints_ordered[k][1]
                
                rospy.loginfo("I m %s and my candidate is " + str(my_candidate), my_id)

                for i in range(len(other_agents)):

                    agent_publishing    = service_call[other_agents[i]](0).ready
                    already_assigned    = service_call[other_agents[i]](my_candidate).is_my_candidate

                    # Controllo se l'agente interrogato non e' ancora nello stato PUBLISHING
                    if agent_publishing is False:
                        other_distance = service_call[other_agents[i]](my_candidate).distance
                        other_distance_dic[other_agents[i]] = other_distance

                    elif already_assigned is True:
                        go_next = True
                        rospy.loginfo('Im %s, my candidate is already assigned', my_id)
                        break

                if go_next is False:

                    if len(other_distance_dic) == 0:
                        my_candidate_assigned = True  

                    else:
                        other_distance_ordered = sorted(other_distance_dic.items(), key=lambda item:item[1])
                        rospy.loginfo('Im %s, calculating other distance form %s: ' + str(other_distance_ordered), my_id, my_candidate)

                        if my_distance < other_distance_ordered[0][1]:
                            my_candidate_assigned = True

                        elif my_distance == other_distance_ordered[0][1] and my_id < other_distance_ordered[0][0]:
                            my_candidate_assigned = True
                
            
                #  Gestione next state
                if my_candidate_assigned is True:
                    next_state = 'PUBLISHING'

                else:
                    k = k + 1
                    next_state = 'DECISION MAKING'

                other_distance_dic = {}
                other_distance_ordered = []
                
            elif curr_state == 'PUBLISHING':

                ready = True
                agent_ready = 0
                result_msg.waypoint_id = my_candidate

                if curr_state != prev_state:
                    pub.publish(result_msg)
                    my_waypoints.append(my_candidate)
                    robot_position = waypoints_avaible[str(my_candidate)]

                    del waypoints_avaible[str(my_candidate)]

                    rospy.loginfo("i m %s and my new position is %s %s in waypoint %s", my_id, robot_position[0], robot_position[1], my_candidate)
                    rospy.loginfo("I'm %s and remaining wapyoints are " + str(list(waypoints_avaible)), my_id)
                
                # Conta gli agenti nello stato PUBLISHING per sincronizzarsi
                for i in range(len(other_agents)):
                    agent_ready = agent_ready + service_call[other_agents[i]](0).ready
                    if agent_ready == (n_robot - 1):
                        next_state = 'FINISH'
            
            elif curr_state == "FINISH":

                if len(waypoints_avaible) > 0:
                    rospy.loginfo("Robot %s ready", my_id)
                    next_state = "CALCULATE DISTANCE"
                    rospy.sleep(1)
                    ready = False
                else:
                    if prev_state != curr_state:
                        rospy.loginfo("Im %s, path finished, my waypoints are: " + str(my_waypoints), my_id)
            
            elif curr_state == "FINISH2":
                if prev_state != curr_state:
                    rospy.loginfo("Im %s in finished 2", my_id)
                

            for i in range(len(other_agents)):

                if callback_recived[other_agents[i]][0] is True:
                    callback_recived[other_agents[i]][0] = False
                    waypoint_to_remove = callback_recived[other_agents[i]][1]
                    del waypoints_avaible[waypoint_to_remove]
                    if next_state == 'DECISION MAKING':
                        rospy.loginfo('Im %s overwrite next_state', my_id)
                        next_state = 'CALCULATE DISTANCE'
                    rospy.loginfo("Im %s remaining waypoints are: " + str(list(waypoints_avaible)), my_id)

            prev_state = curr_state
            curr_state = next_state

        rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_assignment()
    except rospy.ROSInterruptException:
        pass