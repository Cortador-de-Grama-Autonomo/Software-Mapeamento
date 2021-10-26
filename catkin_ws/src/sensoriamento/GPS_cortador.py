import serial #instalar na raspberry
import time
import string
import pynmea2

serialPort = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)

def atualiza_GPS():
    str = serialPort.readline()
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        return msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units, msg.speed, msg.satellites

