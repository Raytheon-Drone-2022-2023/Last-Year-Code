from __future__ import print_function
import time
from dronekit import *
from random import randrange
import sched
import time
import math
from pymavlink import mavutil
import sys
import traceback
import enum

# connect here
vehicle = connect("127.0.0.1:14550", wait_ready=False)


class Units(enum.Enum):
    meters = 1
    yards = 2


def deg_to_rad(deg):
    return deg * (math.pi / 180)


def arm_and_takeoff(targetAlt):
    print("Set mode to GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    print("Arm drone")
    vehicle.arm()
    time.sleep(1)

    print("Taking off to " + str(targetAlt) + " meters")
    vehicle.simple_takeoff(targetAlt)
    while vehicle.location.global_relative_frame.alt < targetAlt * 0.95:
        print('Current height: %d meters\r' % vehicle.location.global_relative_frame.alt, end="")

    print("\nReached target altitude")


def get_coordinates_ahead(distance, units=Units.meters):
    if units is Units.yards:
        distance /= 1.094

    R = 6378.1
    brng = math.radians(vehicle.heading)
    d = float(distance) / 1000.0
    lat1 = math.radians(vehicle.location.global_relative_frame.lat)
    lon1 = math.radians(vehicle.location.global_relative_frame.lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d / R) +
                     math.cos(lat1) * math.sin(d / R) * math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / R) *
                             math.cos(lat1), math.cos(d / R) - math.sin(lat1) * math.sin(lat2))
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    return LocationGlobalRelative(lat2, lon2, vehicle.location.global_relative_frame.alt)


def get_distance_to(location, units=Units.meters):
    lat1 = vehicle.location.global_relative_frame.lat
    lng1 = vehicle.location.global_relative_frame.lon
    lat2 = location.lat
    lng2 = location.lon
    r = 6371000
    dLat = deg_to_rad(lat2 - lat1)
    dLng = deg_to_rad(lng2 - lng1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(deg_to_rad(lat1)) * \
        math.cos(deg_to_rad(lat2)) * math.sin(dLng / 2) * math.sin(dLng / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = r * c
    if units is Units.yards:
        distance *= 1.094

    return distance


def fly_to(location):
    print("Flying to target location")
    vehicle.simple_goto(location)
    while get_distance_to(location) > 1:
        # time.sleep(1)
        print('%d yards away            \r' % get_distance_to(location, Units.yards), end="")

    print('\nReached target location')


def main():
    try:
        arm_and_takeoff(5)

        target_location = get_coordinates_ahead(30, Units.yards)

        fly_to(target_location)

    except KeyboardInterrupt:
        print('Interrupted by user')
    except Exception:
        traceback.print_exc(file=sys.stdout)
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        continue
    print("Landed " + str(round(get_distance_to(target_location, Units.yards), 2)) +
          " yards away from target")
    vehicle.close()
    sys.exit(0)


if __name__ == "__main__":
    main()
