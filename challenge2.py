import math
import camera
import sensors
import flight
from threading import Thread, Lock

FOV = math.pi/2

nav_kill = False
nav_kill_lock = Lock()

const meters_to_degrees = lambda i: i * 360 / 40075000 

def vector_to_coordinates(v):
	v2 = (meters_to_degrees(v[0]), meters_to_degrees(v[1]))
	return (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)

def get_waypoints_for_arena(corner, WE, NS):

	WE = meters_to_degrees(WE)
	NS = -meters_to_degrees(NS)

	waypoints = []

	fov = 2 * math.tan(FOV/2) * sensors.altitude()

	for i in range(corner[0], corner[0] + WE + (fov*WE/math.abs(WE)), (fov*WE/math.abs(WE))):
		
		if i % 2 == 0:
			waypoints.append( (corner[0] + i, corner[1]) )
			waypoints.append( (corner[0] + i, corner[1] + NS) )
		else:
			waypoints.append( (corner[0] + i, corner[1] + NS) )
			waypoints.append( (corner[0] + i, corner[1]) )

	return waypoints

def fly_to_waypoints(points, speed=1):
	#Terminates when it has reached all waypoints or it receives the kill signal
	for l in locations:
		flight.vehicle.simple_goto(l, groundspeed=speed)
		while get_distance_to(l) > 1:
			#Check for the kill signal
			done = False			
			with nav_kill_lock:
				done = nav_kill

			if done: 
				return

def challenge2(corner, WE, NS):

	waypoints = get_waypoints_for_arena(corner, WE, NS)

	#Initialize everything and take off
	camera.initialize()
	flight.initialize()
	flight.arm_and_takeoff(2)

	#Start sweeping the area
	nav_thread = Thread(target=fly_to_waypoints, args=(waypoints, 1))
	nav_thread.start()

	coordinates = None

	#Look for the logo
	while nav_thread.is_alive():
		v = camera.locate_logo_from_image(camera.get_image(), sensors.attitude(), sensors.altitude(), FOV)

		if v is not None: 
			coordinates = vector_to_coordinates(v)

			with nav_kill_lock:
				nav_kill = True

			break

	if coordinates is None:
		#If we didn't find the logo, go back to the corner where you started		
		flight.fly_to_point(corner)
	else:
		#Fly to the logo
		flight.fly_to_point(coordinates)

	#Land and disarm
	nav_thread.join()
	flight.land()
	flight.deinitialize()
	camera.deinitialize()
