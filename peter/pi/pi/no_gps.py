from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

print('Connecting to vehicle')
vehicle = connect('/dev/ttyACM0', wait_ready=False, baud=921600) #USB from PI to PX4
print("Connected")

print("Set Mode to GUIDED")
vehicle.mode = VehicleMode("GUIDED")

print("Arm")
vehicle.arm()
time.sleep(2)

print("Takeoff")
vehicle.simple_takeoff(3)
time.sleep(10)

print("Landing")
vehicle.mode = VehicleMode("LAND")

vehicle.close()
print("Vehicle closed")