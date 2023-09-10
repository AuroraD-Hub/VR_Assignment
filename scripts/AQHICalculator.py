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
import pymap3d as pm
import navpy
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
		update_sampling_data(path, req.n, res)
		res.succes = True
	elif req.command == 'calculate': 
		print('Getting Index')
		res.AQHI = getAQHI(path, req.n)
		res.msg = getHealthMessage(res.AQHI)
		print('AQHI is calculated!')
		res.succes = True
	else:
		print('Error: operation not valid.')
		res.succes = False
	return res
	
def update_sampling_data(path, n, res):
	pose = Pose()
	host = rospy.get_param("/calculator/host") # launch param
	client = airsim.MultirotorClient(ip=host, port=41451)
	data = client.getGpsData(gps_name = "", vehicle_name = "")
	new_position = pm.geodetic2ned(data.gnss.geo_point.latitude, data.gnss.geo_point.longitude, data.gnss.geo_point.altitude, 0, 0, 0)

	if os.path.exists(path) and os.path.getsize(path) > 0:
		with open(path, "r+") as sampling_file:
			sampling_data = json.load(sampling_file)
			# Find index of the last section
			last_section = sampling_data[-1]
			if "PM25" not in last_section:
				print("Key PM25 is not present.")
			if "N" in last_section == 0:
				del last_section
				res.n = 1
			if "N" not in last_section:
				print('Updating file')
				last_section["waypoint"]={}
				new_waypoint = {
					"x": new_position[0],
					"y": new_position[1],
					"z": new_position[2]
				}
				last_section["waypoint"] = new_waypoint
				last_section["N"] = n
				res.n = n+1
				print('File updated')
			else:
				res.n = n
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
	pol_min = min(pol)
	pol_max = max(pol)
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



def getPollutantIndex(aqhi, string):
    # Define AQHI for the specific pollutant
    if string == "O3":
        if aqhi < 0:
            print("Something in getting O3 index went wrong")
            return -1
        if 0 < aqhi < 33:
            return 1
        elif 33 < aqhi < 66:
            return 2
        elif 66 < aqhi < 100:
            return 3
        elif 100 < aqhi < 120:
            return 4
        elif 120 < aqhi < 140:
            return 5
        elif 140 < aqhi < 160:
            return 6
        elif 160 < aqhi < 187:
            return 7
        elif 187 < aqhi < 213:
            return 8
        elif 213 < aqhi < 240:
            return 9
        else:
            return 10
    elif string == "NO2":
        if aqhi < 0:
            print("Something in getting NO2 index went wrong")
            return -1
        if 0 < aqhi < 67:
            return 1
        elif 67 < aqhi < 134:
            return 2
        elif 134 < aqhi < 200:
            return 3
        elif 200 < aqhi < 267:
            return 4
        elif 267 < aqhi < 334:
            return 5
        elif 334 < aqhi < 400:
            return 6
        elif 400 < aqhi < 467:
            return 7
        elif 467 < aqhi < 534:
            return 8
        elif 534 < aqhi < 600:
            return 9
        else:
            return 10
    elif string == "SO2":
        if aqhi < 0:
            print("Something in getting SO2 index went wrong")
            return -1
        if 0 < aqhi < 88:
            return 1
        elif 88 < aqhi < 177:
            return 2
        elif 177 < aqhi < 266:
            return 3
        elif 266 < aqhi < 354:
            return 4
        elif 354 < aqhi < 443:
            return 5
        elif 443 < aqhi < 532:
            return 6
        elif 532 < aqhi < 710:
            return 7
        elif 710 < aqhi < 887:
            return 8
        elif 887 < aqhi < 1064:
            return 9
        else:
            return 10
    elif string == "PM25":
        if aqhi < 0:
            print("Something in getting PM25 index went wrong")
            return -1
        if 0 < aqhi < 11:
            return 1
        elif 11 < aqhi < 23:
            return 2
        elif 23 < aqhi < 35:
            return 3
        elif 35 < aqhi < 41:
            return 4
        elif 41 < aqhi < 47:
            return 5
        elif 47 < aqhi < 53:
            return 6
        elif 53 < aqhi < 58:
            return 7
        elif 58 < aqhi < 64:
            return 8
        elif 64 < aqhi < 70:
            return 9
        else:
            return 10
    elif string == "PM10":
        if aqhi < 0:
            print("Something in getting PM10 index went wrong")
            return -1
        if 0 < aqhi < 16:
            return 1
        elif 16 < aqhi < 33:
            return 2
        elif 33 < aqhi < 50:
            return 3
        elif 50 < aqhi < 58:
            return 4
        elif 59 < aqhi < 66:
            return 5
        elif 66 < aqhi < 75:
            return 6
        elif 75 < aqhi < 83:
            return 7
        elif 83 < aqhi < 91:
            return 8
        elif 91 < aqhi < 100:
            return 9
        else:
            return 10
    else:
	    print(f"Pollutant {string} is not valid.")


def getHealthMessage(AQHI):
	if AQHI>1 and AQHI<3 or AQHI==1 or AQHI==3:
		return "For at-risk individuals: enjoy your usual outdoor activities."
	elif AQHI>4 and AQHI<6 or AQHI==4 or AQHI==6:
		return "For at-risk individuals: adults and children with lung problems, and adults with heart problems, who experience symptoms, should consider reducing strenuous physical activity, particularly outdoors."
	elif AQHI>7 and AQHI<9 or AQHI==7 or AQHI==9:
		return "For at-risk individuals: adults and children with lung problems, and adults with heart problems, should reduce strenuous physical exertion, particularly outdoors, and particularly if they experience symptoms. People with asthma may find they need to use their reliever inhaler more often. Older people should also reduce physical exertion."
	elif AQHI==10:
		return "For at-risk individuals: adults and children with lung problems, adults with heart problems, and older people, should avoid strenuous physical activity. People with asthma may find they need to use their reliever inhaler more often."
	else:
		return "Error in computing the index"
	

if __name__ == '__main__':
    # Instantiate the node manager service and wait.
    calculate()
