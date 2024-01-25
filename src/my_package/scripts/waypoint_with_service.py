#!/usr/bin/env python3
import  rospy
import  math
from    std_msgs.msg        import String, Float64, Int8
from    my_package.srv      import PathLength, PathLengthResponse

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
    response = distance_calculator(robot_position, waypoint_position)
    return response

def waypoint_assignment():
    global enable
    global robot_position
    global waypoints
    global my_id
    
    enable = False
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
    other_id = []                   # Init lista degli ID degli altri robot
    my_waypoints = []               # Lista waypoints assegnati a me
    waypoints_distance = {}         # Dizionario del tipo 'waypoint_id' -> distanza dal waypoint
    service_call = {}               # Dizionario del tipo 'robot_id' -> service client che contatta quell'agente
    other_robot_distance = {}       # Dizionario del tipo 'robot_id' -> distanza di quel robot da un waypoint generico

    # La lista other_id viene riempita con gli id di tutti gli altri agenti
    for id in range(1, n_robot + 1, 1):
        if id is not my_id:
            other_id.append(id)   

    # Topic per debug
    sub = rospy.Subscriber("/chat", String, callback, queue_size=1)
    pub_test = rospy.Publisher("test", Float64, queue_size=2)
    pub = rospy.Publisher("waypoint_assigned", Int8, queue_size=1)

    # Init del service server
    my_service = rospy.Service("path_length", PathLength, handle_path_distance_request)
     
    # Init service client, uno per ogni altro agente
    for i in range(len(other_id)):
        service_name = "/robot_" + str(other_id[i]) + "/path_length"
        rospy.wait_for_service(service_name)
        service_call[other_id[i]] = rospy.ServiceProxy(service_name, PathLength)

    test_msg = Float64()
    result_msg = Int8()
    
    robot_position = [x_0, y_0]

    while not rospy.is_shutdown():
        n_waypoints = len(waypoints)

        if enable is True:
            # Ciclo for che calcola la distanza da tutti gli waypoint
            for i in range(n_waypoints):
                # list() fornisce una lista di tutte le chiavi presenti nel dizionario 'waypoint'
                id_waypoint = list(waypoints)[i]

                # Dizionario: ID waypoint -> distanza dal waypoint
                waypoints_distance[id_waypoint] = distance_calculator(robot_position, waypoints[id_waypoint])
            
            # Lista ordinata del dizionario creato sopra del tipo [['id', distanza], ...]
            waypoint_orderd = sorted(waypoints_distance.items(), key=lambda item:item[1])

            # Id del waypoint con la distanza minore 
            candidate = int(waypoint_orderd[0][0])
            my_distance = waypoint_orderd[0][1]

            # Chiamata al server di tutti gli altri agenti per ottenere la loro distanza dal mio 
            # waypoint candidato
            for i in range(len(other_id)):
                # Richiedo la distanza a other_id[i]
                other_dist = service_call[other_id[i]](candidate).distance
                # Salvo la distanza nel dizionario
                other_robot_distance[other_id[i]] = other_dist
                test_msg.data = other_dist
                pub_test.publish(test_msg)

        rate.sleep()
if __name__ == '__main__':
    try:
        waypoint_assignment()
    except rospy.ROSInterruptException:
        pass
