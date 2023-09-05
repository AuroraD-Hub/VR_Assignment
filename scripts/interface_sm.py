#!/usr/bin/env python

import rospy
import smach
import smach_ros
from VR_Assignment.srv import Spawner_srv, Spawner_srvRequest, Launcher_srv, Launcher_srvRequest, AQHICalculator_srv, AQHICalculator_srvRequest, AQHICalculator_srvResponse

client_spawner = rospy.ServiceProxy('spawner', Spawner_srv)
req_spawner = Spawner_srvRequest()
client_launcher = rospy.ServiceProxy('launcher', Launcher_srv)
req_launcher = Launcher_srvRequest()
client_calculator = rospy.ServiceProxy('calculator', AQHICalculator_srv)
req_calculator = AQHICalculator_srvRequest()
res_calculator = AQHICalculator_srvResponse()

# CAR_MOVING State
class CarMoving(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_reached', 'sampling_done'])
        rospy.loginfo("Car Actor State")
        self.n = 0

        # Create a timer to control if updating pollution file is necessary
        self.update_timer = rospy.Timer(rospy.Duration(2), self.update)

    def update(self, event):
        req_calculator.command = 'update'
        req_calculator.n = self.n
        res_calculator = client_calculator.call(req_calculator)
        self.n = res_calculator.n

    def execute(self, userdata):
        i = input("Whenever you reach your desired position, press 'D' to spawn the drone: ")
        if i == 'D' or i == 'd':
            req_calculator.command = 'calculate'
            req_calculator.n = self.n
            res_calculator = client_calculator.call(req_calculator)
            print("The AQHI is: ", res_calculator.AQHI)
            print(res_calculator.msg)
            req_spawner.vehicle = 'Car'
            client_spawner.call(req_spawner)
            print("Time to sample!")
            return 'goal_reached'
        else:
            print("You chose to reach another position")
            return 'sampling_done'
		
		
# DRONE_FLYING State
class DroneFlying(smach.State):

	def __init__(self):
        #Here, DroneFlying state is initialized.
        	smach.State.__init__(self, outcomes=['goal_reached','sampling_done'])
        	rospy.loginfo("Drone Actor State")
#        	self.n = 0

	# Create a timer to control if updating pollution file is necessary
#        self.update_timer = rospy.Timer(rospy.Duration(2), self.update)

	def update(self, event):
		req_calculator.command = 'update'
		req_calculator.n = self.n
		res_calculator = client_calculator.call(req_calculator)
		self.n = res_calculator.n
	       
	def execute(self, userdata):
		# Start drone controller
		req_launcher.command = 'open'
		client_launcher.call(req_launcher)
		
		i = input("When the drone is landed, press 'C' to move to another position for sampling or 'X' to get the AQHI and close the simulation: ")
		req_launcher.command = 'close'
		client_launcher.call(req_launcher)
		if i == 'C' or i == 'c': 
			req_spawner.vehicle = 'Drone'
			client_spawner.call(req_spawner)
			return 'sampling_done'
		elif i == 'X' or i == 'x':
			# Get AQHI
			req_calculator.command = 'calculate'
			req_calculator.n = self.n
			res_calculator = client_calculator.call(req_calculator)
			print("The AQHI is: ", res_calculator.AQHI)
			print(res_calculator.msg)
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
    
	# Connect to custom services
	rospy.wait_for_service('spawner')
	rospy.wait_for_service('launcher')
	rospy.wait_for_service('calculator')

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
    
    
    
    
    
    
