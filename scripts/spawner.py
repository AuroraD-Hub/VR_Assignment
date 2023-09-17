#!/usr/bin/env python3

import rospy
import setup_path
import airsim
import sys
import os
from os.path import dirname, realpath
import json
import time
import pymap3d as pm
from airsim.types import Pose, Vector3r, KinematicsState
from VR_Assignment.srv import Spawner_srv, Spawner_srvResponse

pos = Pose()
scale = Vector3r()
scale.x_val = 1
scale.y_val = 1
scale.z_val = 1
kinematics = KinematicsState()

def spawner():
	"""
	Spawner node initialized.

	Here, service spawner is instantiated.
	"""
	rospy.init_node("spawner")
	ser = rospy.Service("spawner", Spawner_srv, handle_spawner)
	rospy.spin()
     
def handle_spawner(req):
	"""
	Service callback.    
	The planner executes two different action based on what the state machine needs:
	1) **load**: it loads an available ontology
	2) **exit**: it exits from the current location

	:param req: The request of the service
	:type req: Planner_srv
	"""
	
	res = Spawner_srvResponse()
	host = rospy.get_param("/spawner/host") # launch param
	if req.vehicle == 'Car': # Car Actor State
		print('Looking for Car to destroy...')
		car_api(req.vehicle,host)
		print('Car destroyed!')
		res.succes = True
	elif req.vehicle == 'Drone': # Drone Actor State
		print('Looking for Drone to destroy...')
		drone_api(req.vehicle,host)
		print('Drone destroyed!')
		res.succes = True
	else:
		print('Error: vehicle not valid.')
		res.succes = False
	return res
	
def car_api(vehicle,host):
	# Connect to the AirSim Car simulator
	client = airsim.CarClient(ip=host, port=41451)
	client.confirmConnection()
	
	print("There should be a car: ", client.listVehicles())
	
	# Load JSON path file
	path = dirname(dirname(realpath(__file__)))
	path = path + '/graphs/drone_graph.json'
	with open(path) as drone_path:
		data = json.load(drone_path)
	
	# Save last position
	#pos = client.simGetVehiclePose(vehicle)
	print("Posizione con GetVehiclePose: ", client.simGetVehiclePose(vehicle))
	pos = client.getCarState().kinematics_estimated.position
	print("Car reached position ", (pos.x_val, pos.y_val, pos.z_val))
	for i in range(0,4): ## questo è necessario per poi dare al drone il path da seguire
		data["nodes"][f"p{i}"]["position"][0] = pos.x_val
		data["nodes"][f"p{i}"]["position"][1] = pos.y_val
		data["nodes"][f"p{i}"]["position"][2] = pos.z_val+i*10
		
	with open(path, "w") as save_path:
		json.dump(data, save_path, indent=2)
	
	spawn_new_vehicle(vehicle,host)
	destroy_vehicle(vehicle, client)
	
def drone_api(vehicle,host):
	# Connect to the AirSim simulator
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()
	
	print("There should be a drone: ", client.listVehicles())
	
	# Save last position
	#pos = client.simGetVehiclePose(vehicle)
	data_gps = client.getGpsData(gps_name = "", vehicle_name = "")
	pos = pm.geodetic2ned(data_gps.gnss.geo_point.latitude, data_gps.gnss.geo_point.longitude, data_gps.gnss.geo_point.altitude, 0, 0, 0)
	print("Drone is in position ", (pos[0], pos[1], pos[2]))
	
	spawn_new_vehicle(vehicle,host)
	destroy_vehicle(vehicle, client)
	
	
def destroy_vehicle(vehicle, client):
	if vehicle == 'Car':
		# Load JSON path file
		path = dirname(dirname(realpath(__file__)))
		path = path + '/graphs/drone_graph.json'
		with open(path) as drone_path:
			data = json.load(drone_path)
		
		# Save last position
		pose = client.simGetVehiclePose(vehicle)
		print("Car reached position ", (pose.position.x_val, pose.position.y_val))
		for i in range(0,4):
			data["nodes"][f"p{i}"]["position"][0] = pose.position.x_val
			data["nodes"][f"p{i}"]["position"][1] = pose.position.y_val
			data["nodes"][f"p{i}"]["position"][2] = pose.position.z_val+i*10
			
		with open(path, "w") as save_path:
			json.dump(data, save_path, indent=2)
	elif vehicle == 'Drone':
		# Save last position
		pose = client.simGetVehiclePose(vehicle)		
	else:
		print('Cannot delete unknown vehicles')
	
	# Destroy the vehicle
	client.simDestroyObject(vehicle)
	print("There should not be anything: ", client.listVehicles()) 
		
def spawn_new_vehicle(vehicle,host):
	if vehicle == 'Car': # Spawn a drone
		name = 'Drone'
		vehicle_type = 'SimpleFlight'
		print("Connecting to drone client")
		new_client = airsim.MultirotorClient(ip=host, port=41451)
		new_client.confirmConnection()
		new_client.armDisarm(True)
		new_client.takeoffAsync()
		print("Spawning Drone")
	elif vehicle == 'Drone': # Spawn a car
		name = 'Car'
		vehicle_type = 'PhysXCar'
		print("Connecting to car client")
		new_client = airsim.CarClient(ip=host, port=41451)
		new_client.confirmConnection()
		print("Spawning Car")
	else:
		print('Cannot spawn unknown vehicle')
	
	new_client.simAddVehicle(name, vehicle_type, pos, '')
	vehicle_name = new_client.simSpawnObject(name, vehicle_type, pos, scale, True, True) ## qui UE va in crash e non vedo più nessun comando eseguito dopo sul terminale
	kinematics.orientation = airsim.to_quaternion(0, 0, 0)
	kinematics.position = pos
	new_client.simSetKinematics(kinematics, False, vehicle_name)
	time.sleep(5)
	print(f"There should be a {vehicle}: ", new_client.listVehicles())


if __name__ == '__main__':
    # Instantiate the node manager service and wait.
    spawner()
