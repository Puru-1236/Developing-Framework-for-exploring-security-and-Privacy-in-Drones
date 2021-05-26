from dronekit import connect
import time

# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('127.0.0.1:14550', wait_ready=True)
print('connected With UAV......')
print("Autopilot Firmware version: %s" % vehicle.version)
vehicle.mode=VehicleMode("GUIDED")
