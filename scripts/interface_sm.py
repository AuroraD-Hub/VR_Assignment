#!/usr/bin/env python

import rospy
import smach
import smach_ros
import json
import threading
from VR_Assignment.srv import Spawner_srv, Spawner_srvRequest, Launcher_srv, Launcher_srvRequest

client_spawner = rospy.ServiceProxy('spawner', Spawner_srv)
req_spawner = Spawner_srvRequest()
client_launcher = rospy.ServiceProxy('launcher', Launcher_srv)
req_launcher = Launcher_srvRequest()

# CAR_MOVING State
class CarMoving(smach.State):

	def __init__(self):
        #Here, CarMoving state is initialized.
        	smach.State.__init__(self, outcomes=['goal_reached','sampling_done'])
        	rospy.loginfo("Car Actor State")

	def execute(self, userdata):
		# Start interface
		i = input("Whenever you reach your desired position, press 'D' to spawn the drone: ")
		if i == 'D' or i == 'd': 
			req_spawner.vehicle = 'Car'
			client_spawner.call(req_spawner)
			print("Time to sample!")
			return 'goal_reached'
		else:
			print("You chose to reach another position")
			return 'sampling_done'
		
#		path = '/mnt/c/Users/39348/Documents/Unreal Projects/Assignment/SavedData/PollutionData.json' #path to the Unreal Engine Project folder
#		print("Path trovato")
#		self.pose = client.simGetVehiclePose('Car') # take current car position
#		new_position = [self.pose.position.x_val, self.pose.position.y_val, self.pose.position.z_val]
#		self.update_sampling_data(path, new_position)
#		time.sleep(3)
#		return 'sampling_done'
#
#	def update_sampling_data(self, path, new_position):
#		if os.path.exists(path) and os.path.getsize(path) > 0:
#			with open(path, "r+") as sampling_file:
#				sampling_data = json.load(sampling_file)
#		
#				# Trova l'indice dell'ultima sezione
#				last_section = sampling_data[-1]
#			
#				if "PM25" not in last_section:
#					print("La chiave PM25 non è presente nella sezione")
#				if "waypoint" not in last_section:
#					last_section["waypoint"]={}
#						
#				new_waypoint = {
#					"x": new_position[0],
#					"y": new_position[1],
#					"z": new_position[2]
#				}
#				last_section["waypoint"] = new_waypoint
#		
#				# Sposta il cursore all'inizio del file e sovrascrivi i dati JSON
#				sampling_file.seek(0)
#				json.dump(sampling_data, sampling_file, indent=2)
#				sampling_file.truncate()
#		
#				# Chiudi il file
#				sampling_file.close()
#		else:
#          		print("Il file non esiste o è vuoto")
	
# DRONE_FLYING State
class DroneFlying(smach.State):

	def __init__(self):
        #Here, DroneFlying state is initialized.
        	smach.State.__init__(self, outcomes=['goal_reached','sampling_done'])
        	rospy.loginfo("Drone Actor State")
       
	def execute(self, userdata):
		req_launcher.command = 'open'
		client_launcher.call(req_launcher)
		
		# Start air sampling procedure
#		path = '/mnt/c/Users/39348/Documents/Unreal Projects/Assignment/SavedData/PollutionData.json' 
#		self.pose = client.simGetVehiclePose('Drone') # take current drone position
#		new_position = [self.pose.position.x_val, self.pose.position.y_val, self.pose.position.z_val]
#		self.update_sampling_data(path, new_position)
#		time.sleep(3)
		
		# Start interface
		i = input("When the drone is landed, press 'C' to move to another position for sampling: ")
		req_launcher.command = 'close'
		client_launcher.call(req_launcher)
		if i == 'C' or i == 'c': 
			req_spawner.vehicle = 'Drone'
			client_spawner.call(req_spawner)
			return 'sampling_done'
		else:
			return 'goal_reached'
			
#	def update_sampling_data(self, path, new_position):
#		if os.path.exists(path) and os.path.getsize(path) > 0:
#			with open(path, "r+") as sampling_file:
#				sampling_data = json.load(sampling_file)
#		
#				# Trova l'indice dell'ultima sezione
#				last_section = sampling_data[-1]
#			
#				if "PM25" not in last_section:
#					print("La chiave PM25 non è presente nella sezione")
#				if "waypoint" not in last_section:
#					last_section["waypoint"]={}
#						
#				new_waypoint = {
#					"x": new_position[0],
#					"y": new_position[1],
#					"z": new_position[2]
#				}
#				last_section["waypoint"] = new_waypoint
#		
#				# Sposta il cursore all'inizio del file e sovrascrivi i dati JSON
#				sampling_file.seek(0)
#				json.dump(sampling_data, sampling_file, indent=2)
#				sampling_file.truncate()
#		
#				# Chiudi il file
#				sampling_file.close()
#		else:
#          		print("Il file non esiste o è vuoto")
	

def main():
	"""
	The state machine is initialized and started.

	SMACH is used to create the state machine. It has two states and transitions between
	one another are defined.
	The Introspection Server is also cretated for visualization purpouse.
	At last, the state machine is executed and it runs untill the application is stopped.
	"""
	rospy.init_node("drone_interface_sm", disable_signals=True)
    
	# Connect to custom services
	rospy.wait_for_service('spawner')
	rospy.wait_for_service('launcher')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['container_interface'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('DRONE_FLYING', DroneFlying(),
					transitions={'goal_reached':'DRONE_FLYING',
							'sampling_done':'CAR_MOVING'})
		smach.StateMachine.add('CAR_MOVING', CarMoving(),
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
