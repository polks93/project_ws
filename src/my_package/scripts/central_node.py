#!/usr/bin/env python3

import rospy
from threading import Lock, Thread
import time
from my_package.srv import NegotiateArea, NegotiateAreaResponse
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
   
class CentralNode:
    def __init__(self):
        # Stato interno
        self.lock = Lock()
        self.pending_requests = []  # Richieste in attesa di essere elaborate
        self.processing_requests = False  # Flag per indicare se il nodo sta elaborando le richieste
        self.assignment_interval = 1 # Intervallo di attesa in secondi

        # Stato delle aree: area_id -> agent_id
        self.area_assignments = {}

        # Risposte per gli agenti: agent_id -> response
        self.agent_responses = {}

        # Aree disponibili: area_id (int) -> area_info (dict)
        self.available_areas = init_boxes()

        # Avvia il servizio di negoziazione
        self.service = rospy.Service('negotiate_area', NegotiateArea, self.handle_negotiation)
        rospy.loginfo("Nodo centrale avviato e pronto a gestire le richieste.")

    def handle_negotiation(self, req):
        with self.lock:
            agent_id = req.agent_id
            area_id = req.area_id
            distance = req.distance

            if area_id == -1:
                # Richiesta di stato da parte dell'agente
                response = self.agent_responses.get(agent_id)
                if response:
                    assigned_response = self.agent_responses.pop(agent_id)
                    # Restituisci la risposta salvata
                    return NegotiateAreaResponse(
                        success=response['success'],
                        assigned_area_id=response['assigned_area_id'],
                        message=response['message']
                    )
                else:
                    # Nessuna assegnazione ancora disponibile
                    return NegotiateAreaResponse(
                        success=False,
                        assigned_area_id=-1,
                        message="Assegnazione non ancora disponibile."
                    )
            else:
                # Richiesta iniziale di assegnazione
                rospy.loginfo(f"Ricevuta richiesta da Agente {agent_id} per Area {area_id} con distanza {distance:.2f}")

                # Controlla se l'area esiste
                if area_id not in self.available_areas:
                    rospy.logwarn(f"Area {area_id} non esiste.")
                    return NegotiateAreaResponse(
                        success=False,
                        assigned_area_id=-1,
                        message=f"Area {area_id} non esiste."
                    )

                # Aggiungi la richiesta alla lista delle richieste in sospeso
                self.pending_requests.append({
                    'agent_id': agent_id,
                    'area_id': area_id,
                    'distance': distance,
                })

                # Se non stiamo già elaborando le richieste, avvia il timer
                if not self.processing_requests:
                    self.processing_requests = True
                    Thread(target=self.assignment_cycle).start()

                # Restituisci una risposta indicante che la richiesta è stata ricevuta
                return NegotiateAreaResponse(
                    success=True,
                    assigned_area_id=-1,
                    message="Request received. Waiting for assignment."
                )

    def assignment_cycle(self):
        rospy.loginfo("Starting assignment cycle")
        time.sleep(self.assignment_interval)
        rospy.loginfo("Assignment cycle ended")

        with self.lock:
            # Copia le richieste da elaborare
            requests_to_process = self.pending_requests.copy()
            # Resetta la lista per le nuove richieste
            self.pending_requests = []
            self.processing_requests = False  # Permette di avviare un nuovo ciclo se arrivano nuove richieste

        # Dizionario per le richieste per area
        area_requests = {}
        for request in requests_to_process:
            area_id = request['area_id']
            if area_id not in area_requests:
                area_requests[area_id] = []
            area_requests[area_id].append(request)

        with self.lock:
            # Processa le richieste per ogni area
            for area_id, requests in area_requests.items():
                # Controlla se l'area è già assegnata
                if area_id in self.area_assignments:
                    assigned_agent_id = self.area_assignments[area_id]
                    rospy.loginfo(f"Area {area_id} già assegnata ad Agente {assigned_agent_id}.")
                    # Invia risposta negativa a tutti gli agenti che hanno richiesto quest'area
                    for request in requests:
                        agent_id = request['agent_id']
                        self.agent_responses[agent_id] = {
                            'success': False,
                            'assigned_area_id': -1,
                            'message': f"Area {area_id} già assegnata."
                        }
                    continue

                # Se più agenti hanno richiesto la stessa area, scegli quello con la distanza minore
                if len(requests) > 1:
                    # Ordina le richieste per distanza
                    requests.sort(key=lambda x: x['distance'])
                    selected_request = requests[0]
                    selected_agent_id = selected_request['agent_id']
                    selected_distance = selected_request['distance']
                    rospy.loginfo(f"Area {area_id} assegnata ad Agente {selected_agent_id} (distanza {selected_distance:.2f}).")
                    # Assegna l'area
                    self.area_assignments[area_id] = selected_agent_id
                    del self.available_areas[area_id]
                    
                    # Invia risposta positiva all'agente selezionato
                    self.agent_responses[selected_agent_id] = {
                        'success': True,
                        'assigned_area_id': area_id,
                        'message': f"Area {area_id} assegnata con successo."
                    }
                    # Invia risposta negativa agli altri agenti
                    for request in requests[1:]:
                        agent_id = request['agent_id']
                        self.agent_responses[agent_id] = {
                            'success': False,
                            'assigned_area_id': -2,
                            'message': f"Area {area_id} assegnata ad Agente {selected_agent_id}."
                        }
                else:
                    # Solo un agente ha richiesto l'area
                    request = requests[0]
                    agent_id = request['agent_id']
                    rospy.loginfo(f"Area {area_id} assegnata ad Agente {agent_id}.")
                    # Assegna l'area
                    self.area_assignments[area_id] = agent_id
                    del self.available_areas[area_id]
                    # Invia risposta positiva
                    self.agent_responses[agent_id] = {
                        'success': True,
                        'assigned_area_id': area_id,
                        'message': f"Area {area_id} assegnata con successo."
                    }

if __name__ == '__main__':
    rospy.init_node('central_node')
    central_node = CentralNode()
    rospy.spin()