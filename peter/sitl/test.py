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
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
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


def fly_forward(meters=100):
    meters = int(meters)
    print('flying forward for ' + str(meters) + ' meters')
    startLat = vehicle.location.global_relative_frame.lat
    startLng = vehicle.location.global_relative_frame.lon
    pitch = -(meters/100*25)
    print('Start pitch: ' + str(pitch) + ' degrees')
    duration = 1
    startTime = time.time()
    while True:
        currLat = vehicle.location.global_relative_frame.lat
        currLng = vehicle.location.global_relative_frame.lon
        distance = get_distance(startLat, startLng, currLat, currLng)
        set_attitude(0, pitch, 0, 0.5, duration)
        if distance >= meters:
            set_attitude(0, 0, 0, 0, duration)  # stop
            print('Stopping')
            while(vehicle.airspeed > 0.1):
                time.sleep(1)
            print('Stopped at ' + str(get_distance(startLat, startLng, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)) + ' meters')
            break
        elif (meters - distance) <= 5 * vehicle.airspeed:
            pitch = pitch * (1-(distance/meters))
            print('     Adjusting pitch to : ' + str(pitch) + ' degrees')
        print('Distance to go: ' + str(meters - distance) + ' meters')
        print('     Airspeed: ' + str(vehicle.airspeed))
        print('     Altitude: ' + str(vehicle.location.global_relative_frame.alt) + ' meters')
        print('     Pitch: ' + str(vehicle.attitude.pitch) + ' degrees')
    print('Time elapsed: ' + str(time.time() - startTime) + ' seconds')


def get_distance(lat1, lng1, lat2, lng2):
    r = 6371000
    dLat = deg_to_rad(lat2-lat1)
    dLng = deg_to_rad(lng2-lng1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(deg_to_rad(lat1)) * \
        math.cos(deg_to_rad(lat2)) * math.sin(dLng/2) * math.sin(dLng/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return r * c


def deg_to_rad(deg):
    return deg * (math.pi/180)


def main():
    try:
        arm_and_takeoff(10)
        while True:
            targetDistance = input("How far?: ")
            fly_forward(targetDistance)
            decision = input("Go again? [y/n]: ")
            if not decision == 'y':
                break
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
