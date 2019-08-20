# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
import pigpio
import serial
import time
import traceback

import BMX055
import BME280
import Capture
import GPS
import IM920
import Melting
import Motor
import TSL2561

if __name__ == "__main__":
    try:
        GPS.openGPS()

        # --- Motor Check --- #
        Motor.motor(0, 0, 1)
        Motor.motor(50, 0, 1)
        Motor.motor(0, 50, 1)
        Motor.motor(-50, 0, 1)
        Motor.motor(0, -50, 1)
        Motor.motor(0, 0, 1)

        # --- Melting Check --- #
        Melting.Melting(5)

        # --- Sensor Check --- #
        for i in range(10): #BME280
            print(BME280.bme280_read())
            time.sleep(1)
        for i in range(10): #BMX055
            print(BMX055.bmx055_read())
            time.sleep(1)
        for i in range(10): #GPS
            print(GPS.readGPS())
            time.sleep(1)
        for i in range(10): #TSL2561
            print(TSL2561.readLux())
            time.sleep(1)
        for i in range(10): #Camera
            print(Capture.Capture("photo/"))
            time.sleep(1)
        for i in range(10): #IM920
            IM920.Send("P" + str(i))
            time.sleep(1)
    except:
        Motor.motor(0, 0, 1)
        GPS.closeGPS()
        print(traceback.format_exc())