#!/usr/bin/env python

import rospy
import airsim
import sys
import os
import time
from os.path import dirname, realpath
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import json

def move_drone():
	# Initialize node
	rospy.init_node('drone_controller', anonymous=True)

	# Initialize AirSim client
	host = rospy.get_param("/launcher/host") # launch param
	client = airsim.MultirotorClient(ip='172.23.32.1', port=41451)
	client.confirmConnection()    
	client.enableApiControl(True)    
	client.armDisarm(True)
	client.takeoffAsync().join()
	
	# Get the waypoints
	path = dirname(dirname(realpath(__file__)))
	path = path + '/graphs/drone_graph.json'
	with open(path) as drone_path:
		data = json.load(drone_path)

	waypoints = []
	for i in range(1,4):
		waypoints.append(airsim.Vector3r(data["nodes"][f"p{i}"]["position"][0],data["nodes"][f"p{i}"]["position"][1],data["nodes"][f"p{i}"]["position"][2]))
	
	with open(path, "w") as save_path:
		json.dump(data, save_path, indent=2)

	# Move the drone to each waypoint
	for waypoint in waypoints:
		print("Going towards: ", waypoint)
		client.moveToZAsync(waypoint.z_val, 1.0)
		time.sleep(5)
		
	# Land the drone
	client.hoverAsync()   
	client.landAsync()

	# Chiudi la connessione al client di AirSim
	client.reset()
	client.enableApiControl(False)

if __name__ == '__main__':
    try:
        move_drone()
    except rospy.ROSInterruptException:
        pass


