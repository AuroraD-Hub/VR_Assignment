def main():
	# Get the names of all the spawned drones
	drone_names = rospy.get_param("~drone_name")
	# Disarm all the drones at start
	for i in range(0,len(drone_names))
		pub = rospy.Publisher(f"/airsim_node/{drone_names[i]}/halt", Bool, queue_size=1)
		pub.publish(True)
	
	# Move the base through waypoints
	
	# Arm the drones and start drone_coverage 
