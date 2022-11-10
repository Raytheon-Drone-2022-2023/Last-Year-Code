from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import sched
import time
import math
from pymavlink import mavutil
import sys
import traceback

import argparse
parser = argparse.ArgumentParser(
    description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

if not vehicle.armed:
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.simple_takeoff(10)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= 10 * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0,
                 thrust=0.0, duration=0):
    msg = vehicle.message_factory.set_attitude_target_encode(0, 0, 0, 0b00000000, to_quaternion(
        roll_angle, pitch_angle), 0, 0, math.radians(yaw_rate), thrust)
    if duration != 0:
        modf = math.modf(duration)
        time.sleep(modf[0])
        for x in range(0, int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def time_to_speed(targetSpeed, pitchAngle):
    set_attitude(0, pitchAngle, 0, 1, 1)
    while vehicle.airspeed < targetSpeed*0.95:
        set_attitude(0, pitchAngle, 0, 1, 1)
        continue
    return


while True:
    targetSpeed = input("target speed: ")
    pitchAngle = input("pitch angle: ")
    print('Starting')
    startTime = time.time()
    time_to_speed(targetSpeed, pitchAngle)
    print('Got to ' + str(targetSpeed) + ' in ' +
          str(time.time() - startTime) + ' seconds')
    set_attitude(0, 0, 0, 0, 1)
    decision = input("Fly again? [1/0]")
    if not decision == 1:
        break

print("Landing")
vehicle.mode = VehicleMode("LAND")
while vehicle.armed:
    continue
print("Landed")
vehicle.close()
if sitl:
    sitl.stop()
