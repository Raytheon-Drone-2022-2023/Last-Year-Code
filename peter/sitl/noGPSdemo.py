from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import sched, time, math
from pymavlink import mavutil

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
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

def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    thrust = 0.7
    print('Taking off')
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= aTargetAltitude * 0.95:
            print('Reached ' + str(aTargetAltitude) + ' meters')
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = 0.6
        set_attitude(thrust = thrust, duration = 1)
        time.sleep(0.2)

def set_attitude(roll_angle = 20.0, pitch_angle = -20.0, yaw_rate = 20.0, thrust = 0.5, duration = 0):
    msg = vehicle.message_factory.set_attitude_target_encode(0,0,0,0b00000000,to_quaternion(roll_angle, pitch_angle),0,0,math.radians(yaw_rate),thrust)
    
    if duration != 0:
        modf = math.modf(duration)
        time.sleep(modf[0])
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
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

arm_and_takeoff(15)
vehicle.mode = VehicleMode("LAND")
print('Landing')
while vehicle.armed:
    time.sleep(0.1)
print('Landed')
vehicle.close()
