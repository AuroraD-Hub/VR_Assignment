#!/usr/bin/env python

import rospy
import os
import setup_path
import airsim
import time
from drone_coverage_msgs.srv import LoadCoverageGraph
from drone_coverage_msgs.srv import ComputeCoveragePath
#from VR4R_Assignment import CoverageUtils

def start_drone_coverage(drones_name):
	i = input("Do you want to arm the drones? [Y/N] /n To quit the interface press 'Q'.")
	if i == Y or i == y:
		for drone in drones_name:
			client.armDisarm(True, drone)
		print("Drones state: Arm")
		drone_path = "node_start: 'p0' node_goal: 'p2'"
		client_drone_path = rospy.ServiceProxy('/graph_knowledge/compute', ComputeCoveragePath)
		client_drone_path.call(drone_path)
	elif i == Q or i==q:
		print("Closing the interface")
		rospy.signal_shutdown(i)
	else:
		client.armDisarm(False, drone)
		print("Drones state: Disarm")


def start_interface(drones_name):
	# Complete drone control setup
	file_graph = input("Input the path to the .json graph")
	client_file = rospy.ServiceProxy('/graph_knowledge/load_graph', LoadCoverageGraph)
	client_file.call(file_graph)
	
	# Drone coverage control
	start_drone_coverage(drones_name)
	
		
def main():
	# Creating the node for user interface
	rospy.init_node("interface", disable_signals=True)
	
	# Connect to the AirSim simulator
	client = airsim.MultirotorClient()
	client.confirmConnection()
		
	# Get the names of all the spawned drones
	drones_names = client.listVehicles()
	print(drones_name)
	
	# Disarm all the drones at start
	for drone in drones_name:
		client.enableApiControl(True, drone)
		client.armDisarm(False, drone)
	print("Drones state: Disarm")
	
	# Start interface
	start_interface(drones_name)
	
	rospy.spin()
	
if __name__ == "__main__":
	main()
