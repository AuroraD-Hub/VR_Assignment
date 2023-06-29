#!/usr/bin/env python3

import rospy
import os
import airsim
import json
from geometry_msgs.msg import Pose

def formatting_string(data):
	x = data.position.x_val
	y = data.position.y_val
	z = data.position.z_val

def update_path(client, data):
	# Get GPS coordinates:
	gps_data = client.getGpsData(vehicle_name = "Drone1")
	gps_data = formatting_string(gps_data)
	x = gps['gnss']["geo_point"]["latitude"]
	y = gps['gnss']["geo_point"]["longitude"]
	
	for i in range(0,3):
		data["nodes"][f"p{i}"]["position"][0] = x
		data["nodes"][f"p{i}"]["position"][1] = y
		
	with open("./graphs/drone_graph.json") as save_path:
		json.dump(data, save_path, indent=2)

def main():
	# Creating the node for user interface
	rospy.init_node("path")
	
	# Connect to the AirSim simulator
	client = airsim.MultirotorClient(ip="172.23.32.1", port=41451)
	client.confirmConnection()
	
	# Load JSON path file
	with open("../graphs/drone_graph.json") as drone_path:
		data = json.load(drone_path)
	
	# Update (x,y) vehicle position whenever drones are disarmed
	#if client.armDisarm(False, "Drone1"):
	pose = client.simGetVehiclePose('Drone1')
	#position = formatting_string(position)
	print(pose.position.x_val)
	#update_path(client, data)
	
	rospy.spin()
	
if __name__ == "__main__":
	main()
