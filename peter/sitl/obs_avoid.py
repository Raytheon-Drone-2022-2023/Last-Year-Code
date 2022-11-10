from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from random import randrange
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


def get_distance(lat1, lng1, lat2, lng2):
    r = 6371000
    dLat = deg_to_rad(lat2 - lat1)
    dLng = deg_to_rad(lng2 - lng1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(deg_to_rad(lat1)) * \
        math.cos(deg_to_rad(lat2)) * math.sin(dLng / 2) * math.sin(dLng / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c


def deg_to_rad(deg):
    return deg * (math.pi / 180)


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def mission(distanceMeters=30, alt=10, speed=3):
    R = 6378.1
    brng = math.radians(vehicle.heading)
    d = float(distanceMeters) / 1000.0
    print(str(d))
    lat1 = math.radians(vehicle.location.global_relative_frame.lat)
    lon1 = math.radians(vehicle.location.global_relative_frame.lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / R) * math.cos(lat1),
                             math.cos(d / R) - math.sin(lat1) * math.sin(lat2))
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    start = LocationGlobalRelative(lat1, lon1, alt)
    target = LocationGlobalRelative(lat2, lon2, alt)
    print('target is ' + str(get_distance(lat1, lon1, lat2, lon2)) + ' meters away')
    # print(str(lat1) + ',' + str(lon1))
    # print(str(lat2) + ',' + str(lon2))

    arm_and_takeoff(alt)
    vehicle.simple_goto(target, groundspeed=speed)
    time.sleep(5)
    lastAvoid = -2.5
    dtg = get_distance(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat2, lon2)
    while dtg > 5:
        dtg = get_distance(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat2, lon2)
        time.sleep(1)
        shouldAvoid = (randrange(4) == 1)
        if shouldAvoid is True:
            print('Avoiding object')
            vehicle.simple_goto(target, groundspeed=1)
            time.sleep(2)
            lastAvoid = -lastAvoid
            set_attitude(lastAvoid, 0, 0, 0.6, 1)
            set_attitude(0, 0, 0, 0.6, 1)
            time.sleep(2)
            print('Continuing to target')
            vehicle.simple_goto(target, groundspeed=speed)
    #time.sleep(60)

    # while True:
    #     d_to_target = get_distance(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat2, lon2)
    #     if d_to_target > 1:
    #         continue
    #     else:
    #         print('Reached target ' + str(d_to_target))
    #         time.sleep(10)
    #         print('10 seconds later ' + str(get_distance(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat2, lon2)))
    #         break


def main():
    try:
        mission(distanceMeters=200, alt=10, speed=1)

        # arm_and_takeoff(10)
        # point1 = LocationGlobalRelative(-34.361354, 149.165218, 20)
        # vehicle.simple_goto(point1, groundspeed=3)
        # time.sleep(5)
        # vehicle.simple_goto(point1, groundspeed=1)
        # time.sleep(1)
        # print('move left')
        # start = vehicle.location.global_relative_frame
        # degStart = vehicle.heading
        # set_attitude(-3, 0, -3, 0, 4)
        # set_attitude(0, 0, 0, 0, 2)
        # end = vehicle.location.global_relative_frame
        # degEnd = vehicle.heading
        # print('continue forward after moving ' + str(get_distance(start.lat, start.lon, end.lat, end.lon)) + ' meters left')
        # vehicle.simple_goto(point1, groundspeed=3)
        # time.sleep(20)

    except KeyboardInterrupt:
        print('Interrupted')
    except Exception:
        traceback.print_exc(file=sys.stdout)
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        continue
    print('Landed')
    vehicle.close()
    if sitl:
        sitl.stop()
    sys.exit(0)


if __name__ == "__main__":
    main()
