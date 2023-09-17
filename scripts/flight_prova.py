#!/usr/bin/env python

import rospy
import airsim
import sys
import os
from os.path import dirname, realpath
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import json

def move_drone():
	# Inizializza il nodo ROS
	rospy.init_node('drone_controller', anonymous=True)

	# Inizializza il client di AirSim
	#host = rospy.get_param("/launcher/host") # launch param
	client = airsim.MultirotorClient(ip='172.23.32.1', port=41451)
	client.confirmConnection()    
	client.enableApiControl(True)    
	client.armDisarm(True)
	client.takeoffAsync().join()

	# Ottieni la posizione corrente del drone
	drone_position = client.getMultirotorState().kinematics_estimated.position

	# Definisci una serie di waypoint
	#waypoints = [
	#    airsim.Vector3r(drone_position.x_val + 12.0, drone_position.y_val, drone_position.z_val),
	#    airsim.Vector3r(drone_position.x_val + 22.0, drone_position.y_val, drone_position.z_val),
	#    airsim.Vector3r(drone_position.x_val + 32.0, drone_position.y_val, drone_position.z_val),
	#    airsim.Vector3r(drone_position.x_val + 42.0, drone_position.y_val, drone_position.z_val),
	#    airsim.Vector3r(drone_position.x_val + 90.0, drone_position.y_val - 10.0, drone_position.z_val)
	#]
	path = dirname(dirname(realpath(__file__)))
	path = path + '/graphs/drone_graph.json'
	with open(path) as drone_path:
		data = json.load(drone_path)

	waypoints = []
	for i in range(0,4):
		waypoints.append(airsim.Vector3r(data["nodes"][f"p{i}"]["position"][0],data["nodes"][f"p{i}"]["position"][1],data["nodes"][f"p{i}"]["position"][2]))
		
	print("Waypoint: ",waypoints)

	with open(path, "w") as save_path:
		json.dump(data, save_path, indent=2)

	for idx, waypoint in enumerate(waypoints):
		# Sposta il drone al waypoint corrente
		client.moveToPositionAsync(waypoint.x_val, waypoint.y_val, waypoint.z_val, 1.0).join()

	# Ottieni la posizione corrente dopo il raggiungimento del waypoint
	#drone_position_after_waypoint = client.getMultirotorState().kinematics_estimated.position
	#print(f"Drone posizione dopo waypoint {idx}: {drone_position_after_waypoint}")

	# Apri il file JSON esistente e carica i dati
	#json_file_path = '/mnt/d/VR_Assignment/Assignment/SavedData/PollutionData.json'  # Sostituisci con il percorso effettivo
	#with open(json_file_path, 'r') as json_file:
	#    data = json.load(json_file)

	# Trova l'ultima sezione "PM25"
	#last_pm25_section = None
	#for section in reversed(data):
	#    if "PM25" in section:
	#        last_pm25_section = section
	#        break

	# Aggiungi il campo "waypoint" con la posizione del drone
	#if last_pm25_section:
	#    last_pm25_section["waypoint"] = {
	#        "x": drone_position_after_waypoint.x_val,
	#        "y": drone_position_after_waypoint.y_val,
	#        "z": drone_position_after_waypoint.z_val
	#    }

	    # Salva il JSON aggiornato nel file
	#    with open(json_file_path, 'w') as json_file:
	#        json.dump(data, json_file, indent=4)

	client.hoverAsync().join()     
	client.landAsync().join()

	# Chiudi la connessione al client di AirSim
	client.reset()
	client.enableApiControl(False)

if __name__ == '__main__':
    try:
        move_drone()
    except rospy.ROSInterruptException:
        pass


