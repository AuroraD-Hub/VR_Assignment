#!/usr/bin/env python3

import rospy
import sys
import os
from os.path import dirname, realpath
import subprocess
import signal
import json
from VR_Assignment.srv import Launcher_srv, Launcher_srvResponse
from drone_coverage_msgs.srv import LoadCoverageGraph, ComputeCoveragePath

def launcher():
	"""
	Spawner node initialized.

	Here, service spawner is instantiated.
	"""
	rospy.init_node("launcher")
	ser = rospy.Service("launcher", Launcher_srv, handle_launcher)
	rospy.spin()

def handle_launcher(req):
	"""
	Service callback.    
	The planner executes two different action based on what the state machine needs:
	1) **load**: it loads an available ontology
	2) **exit**: it exits from the current location

	:param req: The request of the service
	:type req: Planner_srv
	"""
	
	res = Launcher_srvResponse()
	if req.command == 'open': 
		print('Launching AirSim Wapper and Drone Controller terminals...')
		open_terminals()
		print('Terminals opened!')
		res.succes = True
	elif req.command == 'close':
		print('Closing terminals...')
		close_terminals()
		print('Terminals closed!')
		res.succes = True
	else:
		print('Error: vehicle not valid.')
		res.succes = False
	return res
	
def open_terminals():
	# Launch AirSim Wrapper and controller for the drone
	host = rospy.get_param("/launcher/host") # launch param
	command = f"gnome-terminal -e 'roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:={host}'"
	wrapper = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
	
	folder_path = dirname(dirname(dirname(realpath(__file__))))
	folder_path = folder_path + '/VR4R_Assignment'
	command = f"gnome-terminal --working-directory={folder_path} -e 'python run.py'"
	controller = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
	
	input("Follow the instruction on drone controller terminal, then press any key here.")
       
	# Load JSON graph and path to follow
	graph = dirname(dirname(realpath(__file__)))
	graph = graph + '/graphs/drone_graph.json'
	client_file = rospy.ServiceProxy('/graph_knowledge/load_graph', LoadCoverageGraph)
	client_file.call(graph)
	client_drone_path = rospy.ServiceProxy('/graph_knowledge/compute_path', ComputeCoveragePath)
	client_drone_path.call('p0', 'p0')
	
def close_terminals():
	# Stop the Wrapper and close related terminal
	subprocess.Popen("pkill gnome-terminal", shell=True, stdin=subprocess.PIPE)

if __name__ == '__main__':
    # Instantiate the node manager service and wait.
    launcher()
