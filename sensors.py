#Functions for collecting information from the various sensors

import time
import board
import busio
import adafruit_lidarlite
from flight import vehicle

#Get the current lidar reading
def lidar():
	sensor = adafruit_lidarlite.LIDARLite(busio.I2C(board.SCL, board.SDA))

	return sensor.distance

def attitude():
	return vehicle.attitude

def altitude():
	return vehicle.altitude

def location():
	return vehicle.gps_0

def heading():
	return vehicle.heading
