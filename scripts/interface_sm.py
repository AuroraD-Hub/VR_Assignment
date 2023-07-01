#!/usr/bin/env python

import rospy
import sys
import os
from os.path import dirname, realpath
import setup_path
import airsim
import smach
import smach_ros
import time
import json
from geometry_msgs.msg import Pose
from drone_coverage_msgs.srv import LoadCoverageGraph, ComputeCoveragePath

# CAR_MOVING State
class CarMoving(smach.State):

	def __init__(self):
        #Here, CarMoving state is initialized.
        	smach.State.__init__(self, outcomes=['goal_reached','sampling_done'])
        	rospy.loginfo("Car Actor State")
        	self.pose = Pose()
       
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
		i = input("Whenever you reach your desired position, press 'D' to arm the drone: ")
		if i == 'D' or i == 'd': 
			self.pose = client.simGetVehiclePose('PhysXCar')
			self.update_path(path, client, data, self.pose) 
			client.simDestroyObject('PhysXCar')
			print("There should not be anything: ", client.listVehicles())
			client.simAddVehicle('Multirotor', 'SimpleFlight', self.pose)
			print("There should be a drone: ", client.listVehicles())
			return 'goal_reached'
		else:
			#"./graphs/drone_graph.json"
			with open(path, "w") as close_file:
				json.dump(data, close_file, indent=2)
			return 'sampling_done'
		
	def update_path(self, path, client, data, pose):
	# update (x,y) vehicle position
		print("Car reached position ", (pose.position.x_val, pose.position.y_val), ".")
		for i in range(0,4):
			data["nodes"][f"p{i}"]["position"][0] = pose.position.x_val
			data["nodes"][f"p{i}"]["position"][1] = pose.position.y_val
			data["nodes"][f"p{i}"]["position"][2] = pose.position.z_val+i*10
			
		with open(path, "w") as save_path:
			json.dump(data, save_path, indent=2)
	
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
		if i == 'C' or i == 'c': 
			self.pose = client.simGetVehiclePose('Multirotor')
			print("There should not be anything: ", client.listVehicles())
			client.simDestroyObject('Multirotor')
			client.simAddVehicle('Car', 'PhysXCar', self.pose)
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
