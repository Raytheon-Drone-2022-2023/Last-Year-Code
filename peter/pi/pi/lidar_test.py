import time
import board
import busio
import adafruit_lidarlite

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lidarlite.LIDARLite(i2c)

while True:
    try:
        print(str(sensor.distance) + "     \r", end="")
    except RuntimeError as e:
        print(e)
    time.sleep(0.01)