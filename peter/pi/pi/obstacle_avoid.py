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
import board
import busio
import adafruit_lidarlite

class Units(enum.Enum):
    meters = 1
    yards = 2

def deg_to_rad(deg):
    return deg * (math.pi / 180)



i2c = busio.I2C(board.SCL, board.SDA)
lidar = adafruit_lidarlite.LIDARLite(i2c)

vehicle = connect('/dev/ttyACM0', wait_ready=False, baud=921600)

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
    #return LocationGlobalRelative(lat2, lon2, vehicle.location.global_relative_frame.alt)
    return LocationGlobalRelative(lat2, lon2, 2)



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


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame relative to current heading
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def obstacle_avoid(target_location):
    vehicle.groundspeed = 1
    last_right = True
    while get_distance_to(target_location) > 1:
        obstacle = False
        try:
            for ms in range(0,100): # search for obstacle for 1 second
                if lidar.distance < 150:
                    obstacle = True
                    print("Obstacle detected")
                    break
                time.sleep(0.01)
        except RuntimeError as e:
            print(e)
        if obstacle: # avoid obstacle alternating right and left
            if last_right:
                print("Avoiding left")
                send_ned_velocity(0, -2, 0, 1)
                last_right = False
            else:
                print("Avoiding right")
                send_ned_velocity(0, 2, 0, 1)
                last_right = True
        else: # continue to destination
            print("Continuing to target")
            vehicle.simple_goto(target_location, groundspeed=1)


def main():
    try:
        time.sleep(2)
        target_location = get_coordinates_ahead(10, Units.yards)

        arm_and_takeoff(2)

        obstacle_avoid(target_location)

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
