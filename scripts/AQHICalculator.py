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
		AQHI = getAQHI(path, n)
		msg = getHealthMessage(AQHI)
		res.AQHI = AQHI
		res.msg = msg
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
			if n == 0:
				del last_section
			else:
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
  		
def getAQHI(path, n):
	# Get pollution level for every pollutant
	O3_NC = getNowCast(path, n, "O3")
	NO2_NC = getNowCast(path, n, "NO2")
	SO2_NC = getNowCast(path, n, "SO2")
	PM25_NC = getNowCast(path, n, "PM25")
	PM10_NC = getNowCast(path, n, "PM10")
	# Get corresponding indices
	AQHI = [getPollutantIndex(O3_NC, "O3"), getPollutantIndex(NO2_NC, "NO2"), getPollutantIndex(SO2_NC, "SO2"), getPollutantIndex(PM25_NC, "PM25"), getPollutantIndex(PM10_NC, "PM10")]
	
	return max(AQHI) # return the maximum among all the indices
	
def getNowCast(path, n, string):
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
	
def getPollutantIndex(aqhi, string)
	# Define AQHI for the specific pollutant
	match string:
		case "O3":
			if aqhi<0:
				print("Something in getting O3 index went wrong")
				return -1
			if aqhi>0 and aqhi<33:
				return 1
			elif aqhi>33 and aqhi<66:
				return 2
			elif aqhi>66 and aqhi<100:
				return 3
			elif aqhi>100 and aqhi<120:
				return 4
			elif aqhi>120 and aqhi<140:
				return 5
			elif aqhi>140 and aqhi<160:
				return 6
			elif aqhi>160 and aqhi<187:
				return 7
			elif aqhi>187 and aqhi<213:
				return 8
			elif aqhi>213 and aqhi<240:
				return 9
			else:
				return 10
		case "NO2":
			if aqhi<0:
				print("Something in getting NO2 index went wrong")
				return -1
			if aqhi>0 and aqhi<67:
				return 1
			elif aqhi>67 and aqhi<134:
				return 2
			elif aqhi>134 and aqhi<200:
				return 3
			elif aqhi>200 and aqhi<267:
				return 4
			elif aqhi>267 and aqhi<334:
				return 5
			elif aqhi>334 and aqhi<400:
				return 6
			elif aqhi>400 and aqhi<467:
				return 7
			elif aqhi>467 and aqhi<534:
				return 8
			elif aqhi>534 and aqhi<600:
				return 9
			else:
				return 10
		case "SO2":
			if aqhi<0:
				print("Something in getting SO2 index went wrong")
				return -1
			if aqhi>0 and aqhi<88:
				return 1
			elif aqhi>88 and aqhi<177:
				return 2
			elif aqhi>177 and aqhi<266:
				return 3
			elif aqhi>266 and aqhi<354:
				return 4
			elif aqhi>354 and aqhi<443:
				return 5
			elif aqhi>443 and aqhi<532:
				return 6
			elif aqhi>532 and aqhi<710:
				return 7
			elif aqhi>710 and aqhi<887:
				return 8
			elif aqhi>887 and aqhi<1064:
				return 9
			else:
				return 10
		case "PM25":
			if aqhi<0:
				print("Something in getting PM25 index went wrong")
				return -1
			if aqhi>0 and aqhi<11:
				return 1
			elif aqhi>11 and aqhi<23:
				return 2
			elif aqhi>23 and aqhi<35:
				return 3
			elif aqhi>35 and aqhi<41:
				return 4
			elif aqhi>41 and aqhi<47:
				return 5
			elif aqhi>47 and aqhi<53:
				return 6
			elif aqhi>53 and aqhi<58:
				return 7
			elif aqhi>58 and aqhi<64:
				return 8
			elif aqhi>64 and aqhi<70:
				return 9
			else:
				return 10
		case "PM10":
			if aqhi<0:
				print("Something in getting PM10 index went wrong")
				return -1
			if aqhi>0 and aqhi<16:
				return 1
			elif aqhi>16 and aqhi<33:
				return 2
			elif aqhi>33 and aqhi<50:
				return 3
			elif aqhi>50 and aqhi<58:
				return 4
			elif aqhi>59 and aqhi<66:
				return 5
			elif aqhi>66 and aqhi<75:
				return 6
			elif aqhi>75 and aqhi<83:
				return 7
			elif aqhi>83 and aqhi<91:
				return 8
			elif aqhi>91 and aqhi<100:
				return 9
			else:
				return 10

def getHealthMessage(AQHI):
	if AQHI>1 and AQHI<3 or AQHI=1 or AQHI=3:
		return "For at-risk individuals: enjoy your usual outdoor activities."
	elif AQHI>4 and AQHI<6 or AQHI=4 or AQHI=6:
		return "For at-risk individuals: adults and children with lung problems, and adults with heart problems, who experience symptoms, should consider reducing strenuous physical activity, particularly outdoors."
	elif AQHI>7 and AQHI<9 or AQHI=7 or AQHI=9:
		return "For at-risk individuals: adults and children with lung problems, and adults with heart problems, should reduce strenuous physical exertion, particularly outdoors, and particularly if they experience symptoms. People with asthma may find they need to use their reliever inhaler more often. Older people should also reduce physical exertion."
	elif AQHI=10:
		return "For at-risk individuals: adults and children with lung problems, adults with heart problems, and older people, should avoid strenuous physical activity. People with asthma may find they need to use their reliever inhaler more often."
	else:
		return "Error in computing the index"
	

if __name__ == '__main__':
    # Instantiate the node manager service and wait.
    calculate()
