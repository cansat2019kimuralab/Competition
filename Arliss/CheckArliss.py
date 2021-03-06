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
import Other

pi = pigpio.pi()

bme280Log = "log/checkBME280Log.txt"
bmx055Log = "log/checkBMX055Log.txt"
gpsLog = "log/checkGSLog.txt"
captureLog = "log/checkCaptureLog.txt"
tsl2561Log = "log/checkTSL2561Log.txt"


if __name__ == "__main__":
	try:
		pi.write(22, 1)
		GPS.openGPS()
		BME280.bme280_setup()
		BME280.bme280_calib_param()
		BMX055.bmx055_setup()
		TSL2561.tsl2561_setup()

		for i in range(10):
			gpsData = GPS.readGPS()
			luxData = TSL2561.readLux()
			bmeData = BME280.bme280_read()
			bmxData = BMX055.bmx055_read()
			photo = Capture.Capture("photo/photo")
			IM920.Strt("1")
			com1Data = IM920.Send("P" + str(1))
			IM920.Strt("2")
			com2Data = IM920.Send("P" + str(2))
			print("GPS\t\t", gpsData)
			print("TSL2561\t\t", luxData)
			print("BME280\t\t", bmeData)
			print("BMX055\t\t", bmxData)
			print("photo\t\t", photo)
			print("fast mode\t", com1Data)
			print("distance mode\t", com2Data)
			print("\n\n")

		"""
		for j in range(1):
			pi.write(22, 1)
			GPS.openGPS()
			BME280.bme280_setup()
			BME280.bme280_calib_param()
			BMX055.bmx055_setup()
			TSL2561.tsl2561_setup()

			# --- Melting Check --- #
			print("\n\nMelting Start")
			Melting.Melting(5)
			time.sleep(10)

			# --- Motor Check --- #
			print("\n\nMotor Check Start")
			Motor.motor(0, 0, 1)
			print(Motor.motor(50, 0, 1))
			print(Motor.motor(0, 50, 1))
			print(Motor.motor(-50, 0, 1))
			print(Motor.motor(0, -50, 1))
			Motor.motor(0, 0, 1)

			# --- Sensor Check --- #
			print("\n\nPressure Sensor Test")
			for i in range(10): #BME280
				data = BME280.bme280_read()
				Other.saveLog(bme280Log, data)
				print(data)
				time.sleep(1)

			print("\n\n9-Axsis Sensor Test")
			for i in range(10): #BMX055
				data = BMX055.bmx055_read()
				Other.saveLog(bmx055Log, data)
				print(data)
				time.sleep(1)

			print("\n\nGPS Test")
			for i in range(10): #GPS
				data = GPS.readGPS()
				Other.saveLog(gpsLog, data)
				print(data)
				time.sleep(1)

			print("\n\nLux Sensor Test")
			for i in range(10): #TSL2561
				data = TSL2561.readLux()
				Other.saveLog(tsl2561Log, data)
				print(data)
				time.sleep(1)

			print("\n\nCapture Test")
			for i in range(10): #Camera
				data = Capture.Capture("photo/photo")
				Other.saveLog(captureLog, data)
				print(data)
				time.sleep(1)

			print("\n\nCommunication Test")
			IM920.Strt("1")
			for i in range(10): #IM920
				data = IM920.Send("P" + str(i))
				print("fastmode\t"+str(data))
			IM920.Strt("2")
			for i in range(10): #IM920
				data = IM920.Send("P" + str(i))
				print("distancemode\t"+str(data))
		"""
	except:
		pi.write(17, 0)
		Motor.motor(0, 0, 1)
		GPS.closeGPS()
		print(traceback.format_exc())
