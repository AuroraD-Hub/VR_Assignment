#!/usr/bin/env python3

import rospy
import setup_path
import airsim
import sys
import os
import time
import math
import json
import threading
from airsim.types import Pose
from VR_Assignment.srv import AQHICalculator_srv, AQHICalculator_srvResponse

def calculate():
	"""
	Spawner node initialized.

	Here, service spawner is instantiated.
	"""
	rospy.init_node("AQHI_calculator")
	ser = rospy.Service("calculator", AQHICalculator_srv, handle_calculator)
	rospy.spin()
     
def handle_calculator(req):
	"""
	Service callback.    
	The planner executes two different action based on what the state machine needs:
	1) **load**: it loads an available ontology
	2) **exit**: it exits from the current location

	:param req: The request of the service
	:type req: Planner_srv
	"""
	
	res = AQHICalculator_srvResponse()
	path = '/mnt/c/Users/39348/Documents/Unreal Projects/Assignment/SavedData/PollutionData.json' #path to the Unreal Engine Project folder
	if req.command == 'update': 
		print('Updating file')
		update_sampling_data(path, req.n)
		print('File updated')
		res.succes = True
	elif req.command == 'calculate': 
		print('Get Index')
		AQHI = max([getAQHI(path,req.n,"CO"), getAQHI(path,req.n,"O3"), getAQHI(path,req.n,"SO2"), getAQHI(path,req.n,"NO2"), getAQHI(path,req.n,"PM25"), getAQHI(path,req.n,"PM10")])
		res.AQHI = AQHI
		print('AQHI is calculated!')
		res.succes = True
	else:
		print('Error: operation not valid.')
		res.succes = False
	return res
	
def update_sampling_data(path, n):
	pose = Pose()
	client = airsim.CarClient(ip="172.23.32.1", port=41451) # change into Multirotor
	pose = client.simGetVehiclePose('Car') # change into Drone
	new_position = [pose.position.x_val, pose.position.y_val, pose.position.z_val]

	if os.path.exists(path) and os.path.getsize(path) > 0:
		with open(path, "r+") as sampling_file:
			sampling_data = json.load(sampling_file)
	
			# Find index of the last section
			last_section = sampling_data[-1]
		
			if "PM25" not in last_section:
				print("Key PM25 is not present.")
			if "waypoint" not in last_section:
				last_section["waypoint"]={}
					
			new_waypoint = {
				"x": new_position[0],
				"y": new_position[1],
				"z": new_position[2]
			}
			last_section["waypoint"] = new_waypoint
			last_section["N"] = n
	
			# Move the cursor to the start of the file and update the file
			sampling_file.seek(0)
			json.dump(sampling_data, sampling_file, indent=2)
			sampling_file.truncate()
	
			# Close the file
			sampling_file.close()
	else:
  		print("File does not exist or is empty.")
  		
def getAQHI(path, n, string):
	# Retrive data from file
	with open(path, "r") as path:
		data = json.load(path)
		
	# Calculate the index
	pol=[]
	num = 0
	den = 0
	for i in data:
		pol.append(i.get(string, 0))
	w_star = pol_min/pol_max
	if w_star > 0.5:
		w = w_star
	else:
		w = 0.5
	for i in range(0,n):
		num = num + math.pow(w,i)*pol[i]
		den = den + math.pow(w,i)
	pol_NowCast = num/den
	
	return pol_NowCast

if __name__ == '__main__':
    # Instantiate the node manager service and wait.
    calculate()
