# cd ~/ardupilot/ArduCopter
# ../Tools/autotest/sim_vehicle.py -L University -M --console --map --out=udpbcast:127.0.0.1:14550

from dronekit import *

import socket
import argparse 
import math
import pygame

parser = argparse.ArgumentParser()
parser.add_argument('--connect')
args = parser.parse_args()

def sitl_connect(connection_string):
	try: 
		vehicle = connect(connection_string,115200, wait_ready = True, heartbeat_timeout=15)
		print("Virtual Copter ready")
		return vehicle
		
	except socket.error:
		print('No server exists!')

	except OSError:
		print('No serial exists')

	except APIException:
		print ('timeout')

	except Exception:
		print('some other error')

if args.connect:
	print(f"Connection String: {args.connect}")
	connection_string = args.connect
else:
	print("Defaulting to IP: /dev/ttyS0")
	connection_string = '127.0.0.1:14550'

vehicle = sitl_connect(connection_string)

@vehicle.on_attribute('mode')
def mode_callback(self, attr_name, value):
	print(f">> Mode Updated: {value}")
	
vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)

Ref_lat = 23.0359623
Ref_lon = 72.551403
Ref_alt = 0

Ref_location = LocationGlobalRelative(Ref_lat,Ref_lon,Ref_alt)

def launch_seq(altitude):
	#Poll until vehicle is ready to arm
	
	while not vehicle.is_armable == True:
		print(f"Arming ...")
		time.sleep(2)
	
	print(f"Ready to Arm: {vehicle.is_armable}\n")
	
	#Poll until vehicle in guided mode and armed
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
		print("Getting ready to take off ...")
		time.sleep(2)
	time.sleep(3)
	print(f"Armed: {vehicle.armed}\n")
	
	time.sleep(1)
    #Call using target altitude
	vehicle.simple_takeoff(altitude)
	
	#Poll until done
	while True:
		print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
		#Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt >= altitude*0.95:
			print("Reached target altitude\n")
			break
		time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
	
	earth_radius=6378137.0
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
	
	newlat=original_location.lat + (dLat*180/math.pi)
	newlon=original_location.lon + (dLon*180/math.pi)
	if type(original_location) is LocationGlobal:
		targetlocation=LocationGlobal(newlat, newlon, original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
		targetlocation=LocationGlobalRelative(newlat, newlon, original_location.alt)
	else:
		raise Exception("Invalid Location object passed")
		
	return targetlocation;
	
def set_home_position(drone_location):
	vehicle.home_location = drone_location
	print('Home Location Set')
	spot = get_spot(Ref_location,drone_location)
	print('The home location spot in ' , spot)
	
def get_distance_metres(aLocation1, aLocation2):
	dlat = (aLocation2.lat - aLocation1.lat) * math.pi/180
	dlong = (aLocation2.lon - aLocation1.lon) * math.pi/180
	lat1 = aLocation1.lat * math.pi/180
	lat2 = aLocation2.lat * math.pi/180
	#return math.sqrt((dlat*dlat)+(dlong*dlong))* 6371000
	a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(lat1) * math.cos(lat2) * math.sin(dlong/2) * math.sin(dlong/2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	return  c * 6371000
	
def get_bearing(aLocation1, aLocation2):
	"""y = math.sin(math.radians(aLocation2.lon) - math.radians(aLocation1.lon)) * math.cos(math.radians(aLocation2.lat))
	x = (math.cos(math.radians(aLocation1.lat))*math.sin(math.radians(aLocation2.lat))) - (math.sin(math.radians(aLocation1.lat))*math.cos(math.radians(aLocation2.lat))*math.cos(math.radians(aLocation2.lon) - math.radians(aLocation1.lon)))
	a = math.atan2(y, x)
	return (((a*180)/math.pi)+360)%360"""
	
	y = math.sin(math.radians(aLocation1.lon) - math.radians(aLocation2.lon)) * math.cos(math.radians(aLocation1.lat))
	x = (math.cos(math.radians(aLocation2.lat))*math.sin(math.radians(aLocation1.lat))) - (math.sin(math.radians(aLocation2.lat))*math.cos(math.radians(aLocation1.lat))*math.cos(math.radians(aLocation1.lon) - math.radians(aLocation2.lon)))
	a = math.atan2(y, x)
	return (((a*180)/math.pi)+180)%360
	
def go_to_location(target_location,groundspeed):
	
	drone_location = vehicle.location.global_relative_frame
	Actual_Dist = get_distance_metres(target_location,drone_location)
	
	vehicle.simple_goto(target_location,groundspeed)

	#Polling untill reached
	while True:
		drone_location = vehicle.location.global_relative_frame
		Current_Dist = get_distance_metres(target_location,drone_location)
		if Current_Dist <= Actual_Dist*0.10:
			print ("Reached Loaction\n")
			break
		time.sleep(2)

def distance_bearing(Location1,Location2):
	print(get_distance_metres(Location1, Location2))
	print(get_bearing(Location1, Location2))
		
def arm_drone():
	while not vehicle.is_armable == True:
		print(f"Arming ...")
		time.sleep(2)
	print(f"Ready to Arm: {vehicle.is_armable}\n")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
		print("Getting ready to take off ...")
		time.sleep(2)
	time.sleep(2)
	print(f"Armed: {vehicle.armed}\n")

def command():
	while True:
		a = input("Enter Command: ")
		a = a.strip()
		if a == 'arm':
			arm_drone()
		elif a == 'launch':
			altitude = int(input('Enter the hieght: '))
			launch_seq(altitude)
		elif a == 'go':
			North = float(input('Enter north dist: '))
			East = float(input('Enter east dist: '))
			Speed = int(input('Enter Speed: '))
			drone_location = vehicle.location.global_relative_frame
			location1 = get_location_metres(drone_location,North,East)
			print(location1)
			print(get_distance_metres(drone_location,location1))
			print(get_bearing(drone_location,location1))
			go_to_location(location1,Speed)
		elif a == 'land':
			vehicle.mode = VehicleMode("LAND")
			time.sleep(2)
		elif a == 'rtl':
			vehicle.mode = VehicleMode("RTL")
			time.sleep(2)
		elif a == 'end':
			break
		elif a == 'location':
			print(vehicle.location.global_relative_frame)
		elif a == 'home location':
			drone_location = vehicle.location.global_frame
			set_home_position(drone_location)
		elif a == 'measure':
			drone_location = vehicle.location.global_relative_frame
			distance_bearing(vehicle.home_location,drone_location)
		else :
			time.sleep(1)

def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,  
            vx, vy, vz, 
            0, 0, 0,   
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_yaw_body(vehicle,heading,direction):
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target_system, target_component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		direction,          # param 3, direction -1 ccw, 1 cw
		1, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	vehicle.send_mavlink(msg)
	vehicle.flush()

def keyboard():
	print("""---------------------------------------------
Keyboard control
Control direction with arrow keys and "w" and "s" keys for altitude
"q" to go back
---------------------------------------------""")
	pygame.init()
	screen = pygame.display.set_mode((600,00))
	
	run = True
	
	while run:
		
		key = pygame.key.get_pressed()
		if key[pygame.K_UP] == True and key[pygame.K_LEFT] == True:
			set_velocity_body(vehicle, 0, -0.7, 0)
			time.sleep(0.005)
		elif key[pygame.K_UP] == True and key[pygame.K_RIGHT] == True:
			set_velocity_body(vehicle, 0, 0.7, 0)
			time.sleep(0.005)
		elif key[pygame.K_LEFT] == True:
			set_yaw_body(vehicle,2,-1)
			time.sleep(0.02)
		elif key[pygame.K_RIGHT] == True:
			set_yaw_body(vehicle,2,1)
			time.sleep(0.02)
		elif key[pygame.K_w] == True:
			set_velocity_body(vehicle, 0, 0, -0.5)
			time.sleep(0.005)
		elif key[pygame.K_s] == True:
			set_velocity_body(vehicle, 0, 0, 0.5)
			time.sleep(0.005)
		elif key[pygame.K_UP] == True:
			set_velocity_body(vehicle, 1 , 0, 0)
		elif key[pygame.K_q] == True:
			pygame.quit()
			break
			time.sleep(1)
	
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False
				
def get_spot(Ref,drone):
	drone_location = drone
	zero_location = Ref
	dist = get_distance_metres(zero_location,drone_location)
	angle = get_bearing(zero_location,drone_location)
	ndist = dist * math.cos((math.pi*angle)/180)
	edist = dist* math.sin((math.pi*angle)/180)
	col = ndist // 2
	row = edist // 2
	print('Column is :',col)
	print('Row is :' , row)
	
	return (col+1) + ((row)*25)
	
	
def algorithm_path():
	drone_location = vehicle.location.global_relative_frame
	spot = get_spot(Ref_location,drone_location)
	print('the home location spot is ', spot)
	path = str(input("Enter the Path from algorithm :"))
	speed = int(input("Enter Speed :"))
	alt = int(input("Enter the alt :"))
	launch_seq(alt)	
	a = len(path) - 1
	last = 0
	present = 0
	while a >= 0:
		b = path[a]
		if b == "*":
			present = a
			if last == 0:
				last = present
			else: 
				spot = (path[present + 1:last])
				print(spot)
				if spot == "###":
					vehicle.mode = VehicleMode("LAND")
					time.sleep(2)
				else:
					spot = int(spot)
					row = spot % 25
					col = spot // 25
					if row == 0:
						row = 25
						col = col - 1
					print ('Row is :' , row)
					print('Column is :',col)
					ndist = (row*2) - 1
					edist = (col*2) - 1
				
					target_location = get_location_metres(Ref_location,ndist,edist)
					go_to_location(target_location, speed)
				
					last = present
		a -= 1
			
while True:
	a = input("""------------------------------------------------------
# Press 1 for Keyboard Control 
# Press 2 for Command Control
# Press 3 for A* algorithm path
# Press 4 to Exit 
------------------------------------------------------
Respond: """)
	if a == '1':
		if not vehicle.armed:
			launch_seq(2)
		keyboard()
	elif a == '2':
		command()
	elif a == '3':
		algorithm_path()
	elif a == '4':
		break
