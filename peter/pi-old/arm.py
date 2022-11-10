from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

#parser = argparse.ArgumentParser()
#parser.add_argument('--connect', default='127.0.0.1:14550')
#args = parser.parse_args()

# Connect to the Vehicle
#print('Connecting to vehicle on: %s' % args.connect)
#vehicle = connect(args.connect, baud=921600, wait_ready=True)


print('Connecting to vehicle')

# PI
#vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

# Mac
#vehicle = connect('/dev/cu.usbserial-0001', wait_ready=True, baud=57600)
vehicle = connect('/dev/cu.usbmodem14601', wait_ready=True)

print("Pre-arm check")
while not vehicle.is_armable:
    print("Waiting for vehicle to initialise...")
    time.sleep(1)

print("Arming")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

print("Armed")
time.sleep(3)

vehicle.mode = VehicleMode("LAND")
vehicle.close()
