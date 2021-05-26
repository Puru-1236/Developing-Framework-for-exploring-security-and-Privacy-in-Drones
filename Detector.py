from dronekit import connect, VehicleMode
import sys
import time


# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

#vehicle.mode=VehicleMode("GUIDED")
if vehicle.mode.name!= 'STABILIZE':
	print('Attacker changed the mode of the drone')
else:
	print("Mode is correct showing according to legitimate user action: %s" % vehicle.mode.name)
#print(" Home Location: %s" % vehicle.home_location)
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print("Local Location: %s" % vehicle.location.local_frame)
print("Local Location: %s" % vehicle.location.local_frame.north)
if vehicle.location.local_frame.north >1:
	for i in range(10):	
		print("Alarm : Attack happened on drone, Position has been changed , need to return to launch position")
	vehicle.mode=VehicleMode("RTL")
