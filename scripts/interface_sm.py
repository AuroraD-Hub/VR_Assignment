#!/usr/bin/env python

import rospy
import sys
import os
from os.path import dirname, realpath
import signal
import setup_path
import airsim
import smach
import smach_ros
import time
import json
from airsim.types import Vector3r, KinematicsState
from geometry_msgs.msg import Pose
from drone_coverage_msgs.srv import LoadCoverageGraph, ComputeCoveragePath

# CAR_MOVING State
class CarMoving(smach.State):

	def __init__(self):
        #Here, CarMoving state is initialized.
        	smach.State.__init__(self, outcomes=['goal_reached','sampling_done'])
        	rospy.loginfo("Car Actor State")
        	self.pose = Pose()
        	self.scale = Vector3r()
        	self.scale.x_val = 1
        	self.scale.y_val = 1
        	self.scale.z_val = 1
        	self.kinematics = KinematicsState()
       
	def execute(self, userdata):
		# Connect to the AirSim simulator
		client = airsim.CarClient(ip="172.23.32.1", port=41451)
		client.confirmConnection()
		client.enableApiControl(True)
		
		print("There should be a car: ", client.listVehicles())	
	       
		# Load JSON path file
		path = dirname(dirname(realpath(__file__)))
		path = path + '/graphs/drone_graph.json'
		with open(path) as drone_path:
			data = json.load(drone_path)
		
		# Start interface
		i = input("Whenever you reach your desired position, press 'D' to spawn the drone: ")
		if i == 'D' or i == 'd': 
			# Destroy the car
			self.pose = client.simGetVehiclePose('Car')
			self.update_path(path, data, self.pose) 
			client.simDestroyObject('Car')
			time.sleep(3)
			print("There should not be anything: ", client.listVehicles())
			# Spawn a drone
			self.spawn_drone(client, self.pose, self.scale, self.kinematics)
			time.sleep(3)
			print("There should be a drone: ", client.listVehicles())
			return 'goal_reached'
		else:
			with open(path, "w") as close_file:
				json.dump(data, close_file, indent=2)
			return 'sampling_done'
		
	def update_path(self, path, data, pose):
	# update (x,y) vehicle position
		print("Car reached position ", (pose.position.x_val, pose.position.y_val))
		for i in range(0,4):
			data["nodes"][f"p{i}"]["position"][0] = pose.position.x_val
			data["nodes"][f"p{i}"]["position"][1] = pose.position.y_val
			data["nodes"][f"p{i}"]["position"][2] = pose.position.z_val+i*10
			
		with open(path, "w") as save_path:
			json.dump(data, save_path, indent=2)
			
	def spawn_drone(self, client, pose, scale, kinematics):
		pawn_path = dirname(dirname(dirname(realpath(__file__))))
		pawn_path = pawn_path + '/AirSim/Plugins/AirSim/Content/Blueprints/BP_FlyingPawn.uasset'
		client.simAddVehicle('Drone', 'SimpleFlight', pose, pawn_path)
		client.simSpawnObject('Drone', 'SimpleFlight', pose, scale, True, True)
		kinematics.orientation = airsim.to_quaternion(0, 0, 0)
		kinematics.position = pose
		client.simSetKinematics(kinematics, False, 'Drone')
	
# DRONE_FLYING State
class DroneFlying(smach.State):

	def __init__(self):
        #Here, DroneFlying state is initialized.
        	smach.State.__init__(self, outcomes=['goal_reached','sampling_done'])
        	rospy.loginfo("Drone Actor State")
        	self.pose = Pose()
       
	def execute(self, userdata):
		# Connect to the AirSim simulator
		client = airsim.MultirotorClient(ip="172.23.32.1", port=41451)
		client.confirmConnection()
		client.enableApiControl(True)
		
		print("There should be a drone: ", client.listVehicles())
		
		# Launch AirSim Wrapper for the drone
		wrapper = os.system("gnome-terminal -e 'roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=172.23.32.1'")
		#Stat interface
		print("Execute './run.py' from VR4R_Assignment")
		input("Done? If so, press any key to continue.")
	       
		# Load JSON graph and path to follow
		graph = dirname(dirname(realpath(__file__)))
		graph = graph + '/graphs/drone_graph.json'
		client_file = rospy.ServiceProxy('/graph_knowledge/load_graph', LoadCoverageGraph)
		client_file.call(graph)
		client_drone_path = rospy.ServiceProxy('/graph_knowledge/compute_path', ComputeCoveragePath)
		client_drone_path.call('p0', 'p0')
		
		# Start interface
		i = input("When the drone is landed, press 'C' to move to another position for sampling: ")
		# Stop the Wrapper and close related terminal
		os.kill(wrapper.getppid(), signal.SIGHUP) # gives error
		if i == 'C' or i == 'c': 
			self.pose = client.simGetVehiclePose('Drone')
			client.simDestroyObject('SimpleFlight')
			time.sleep(3)
			print("There should not be anything: ", client.listVehicles())
			client.simAddVehicle('Car', 'PhysXCar', self.pose)
			time.sleep(3)
			print("There should be a car: ", client.listVehicles())
			return 'sampling_done'
		else:
			return 'goal_reached'
	

def main():
    """
    The state machine is initialized and started.

    SMACH is used to create the state machine. It has two states and transitions between
    one another are defined.
    The Introspection Server is also cretated for visualization purpouse.
    At last, the state machine is executed and it runs untill the application is stopped.
    """
    rospy.init_node("drone_interface_sm", disable_signals=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CAR_MOVING', CarMoving(),
                               transitions={'goal_reached':'DRONE_FLYING',
                                            'sampling_done':'CAR_MOVING'})
        smach.StateMachine.add('DRONE_FLYING', DroneFlying(),
                               transitions={'goal_reached':'DRONE_FLYING',
                                            'sampling_done':'CAR_MOVING'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ASSIGNMENT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
