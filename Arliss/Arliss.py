# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/GoalDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/ReleaseAndLandingDetection')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Calibration')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Control')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Goal')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Running')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Stuck')
sys.path.append('/home/pi/git/kimuralab/Mission')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/IM920')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Motor')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/Other')
sys.path.append('/home/pi/git/kimuralab/Mission')
import binascii
import difflib
import math
import numpy as np
import os
import pigpio
import serial
import time
import traceback

import BMX055
import BME280
import Capture
import Calibration
import goal_detection
import GPS
import IM920
import Land
import Melting
import Motor
import Other
import ParaDetection
import ParaAvoidance
import pidControl
import Release
import RunningGPS
import sendPhoto
import Stuck
import stuckDetection
import TSL2561


phaseChk = 0	#variable for phase Check

# --- variable of time setting --- #
t_start  = 0.0				#time when program started
t_sleep = 3				#time for sleep phase
t_release = 120				#time for release(loopx)
t_land = 210				#time for land(loopy)
t_melt = 5					#time for melting
t_transmit = 30				#time for transmit limit
t_sleep_start = 0			#time for sleep origin
t_release_start = 0			#time for release origin
t_land_start = 0			#time for land origin
t_transmit_start = 0
t_calib_origin = 0			#time for calibration origin
t_paraDete_start = 0
t_takePhoto_start = 0		#time for taking photo
t_goalDete_start = 0
t_stuckDete_start = 0
timeout_calibration = 180	#time for calibration timeout
timeout_parachute = 60
timeout_takePhoto = 30		#time for taking photo timeout
timeout_sendPhoto = 120		#time for sending photo timeout
timeout_goalDete = 300
timeout_stuck = 100

# --- variable for storing sensor data --- #
gpsData = [0.0,0.0,0.0,0.0,0.0]						#variable to store GPS data
bme280Data = [0.0,0.0,0.0,0.0]						#variable to store BME80 data
bmx055data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]	#variable to store BMX055 data

# --- variable for Judgement --- #
lcount = 0			#lux count for release
acount = 0			#press count for release
fcount = 0			#photo count for emergency release
pcount = 0			#press count for land
GAcount = 0
gacount = 0			#GPSheight count for land
mcount = 0			#Magnet count

luxjudge = 0		#for release
pressjudge = 0		#for release and land
magnetlandjudge=0   #for emergency landjudge
gpsjudge = 0		#for land
stuckFlug = 0
stuckThd = 100
PstuckCount = 30
stuckCount = 100
stuckCountThd = 10
LuxThd = 70			#variable for cover para
paraExsist = 0 		#variable for Para Detection    0:Not Exsist, 1:Exsist
goalFlug = -1		#variable for GoalDetection		-1:Not Detect, 0:Goal, 1:Detect
goalBufFlug = -1	#variable for GoalDetection buf
goalArea = 0		#variable for goal area
goalGAP = -1		#variable for goal gap
goalthd = 10000		#variable for goal area thd
H_min = 200			#Hue minimam
H_max = 10			#Hue maximam
S_thd = 130			#Saturation threshold
bomb = 0			#use for goalDete flug

# --- variable for Transmit --- #
mode = 1			#valiable for transmit mode
readmode = 0		#valiable for image read mode
count = 0			#valiable for transmit count
amari = 0

# --- variable for Running --- #
ellipseScale = [-99.79881015576746, 171.782066653816, 1.586018339640338, 0.9521503173796134] #Convert coefficient Ellipse to Circle
disGoal = 100.0						#Distance from Goal [m]
angGoal = 0.0						#Angle toword Goal [deg]
angOffset = -77.0					#Angle Offset towrd North [deg]
gLat, gLon = 35.918181, 139.90799	#Coordinates of That time
nLat, nLon = 0.0, 0.0		  		#Coordinates of That time
nAng = 0.0							#Direction of That time [deg]
relAng = [0.0, 0.0, 0.0]			#Relative Direction between Goal and Rober That time [deg]
rAng = 0.0							#Median of relAng [deg]
mP, mPL, mPR, mPS = 0, 0, 0, 0		#Motor Power
kp = 0.5							#Proportional Gain
stuckMode = [0, 0]					#Variable for Stuck
maxMP = 70							#Maximum Motor Power

# --- variable for Goal Detection --- #
mp_min = 20							#motor power for Low level
mp_max = 60							#motor power fot High level
mp_adj = -3							#adjust motor power
adj_add = 8

# --- variable of Log path --- #
phaseLog =			"/home/pi/log/phaseLog.txt"
sleepLog = 			"/home/pi/log/sleepLog.txt"
releaseLog = 		"/home/pi/log/releaseLog.txt"
landingLog = 		"/home/pi/log/landingLog.txt"
meltingLog = 		"/home/pi/log/meltingLog.txt"
paraAvoidanceLog = 	"/home/pi/log/paraAvoidanceLog.txt"
runningLog = 		"/home/pi/log/runningLog.txt"
goalDetectionLog =	"/home/pi/log/goalDetectionLog.txt"
captureLog = 		"/home/pi/log/captureLog.txt"
missionLog = 		"/home/pi/log/missionLog.txt"
calibrationLog = 	"/home/pi/log/calibrationLog"
sendPhotoLog = 		"/home/pi/log/sendPhotoLog.txt"
stuckLog = 			"/home/pi/log/stuckLog.txt"
errorLog = 			"/home/pi/log/erroLog.txt"

photopath = 		"/home/pi/photo/photo"
photoName =			""

fileCal = 			""	#File Path for Calibration Log

pi = pigpio.pi()	#object to set pigpio


def setup():
	global phaseChk
	pi.set_mode(17,pigpio.OUTPUT)
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)					#IM920	Turn On
	pi.write(17,0)					#Outcasing Turn Off
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

	with open(phaseLog, 'a') as f:
		pass

	#if it is End to End Test, then
	try:
		phaseChk = int(Other.phaseCheck(phaseLog))
	except:
		phaseChk = 0
	#if it is debug
	#phaseChk = 7

def transmitPhoto():
	global t_start
	IM920.Strt(1)
	time.sleep(1)
	takePhoto()
	print("Send Photo")
	sendPhoto.sendPhoto(photoName)
	print("Send Photo Finished")
	Other.saveLog(sendPhotoLog, time.time() - t_start, GPS.readGPS(), photoName)
	IM920.Strt(2)
	time.sleep(1)

def takePhoto():
	global photoName
	photo = ""
	photo = Capture.Capture(photopath)
	Other.saveLog(captureLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), photoName)
	if not(photo == "Null"):
		photoName = photo

def calibration():
	global ellipseScale
	mPL, mPR, mPS = 0, 0, 0
	dt = 0.05
	roll = 0
	time.sleep(1)
	fileCal = Other.fileName(calibrationLog, "txt")

	print("Calibration Start")
	Motor.motor(0, 30, 1)
	t_cal_start = time.time()
	while(math.fabs(roll) <= 600):
		if(time.time() - t_cal_start >= 10):
			calData = ellipseScale
			Motor.motor(0, 0, 1)
			Motor.motor(-60, -60, 2)
			Other.saveLog(fileCal, time.time() - t_start, "Calibration Failed")
			break

		mPL, mPR, mPS, bmx055data = pidControl.pidSpin(300, 1.0, 1.1, 0.2, dt)
		with open(fileCal, 'a') as f:
			for i in range(6, 8):
				#print(str(bmx055data[i]) + "\t", end="")
					f.write(str(bmx055data[i]) + "\t")
			#print()
			f.write("\n")
		roll = roll + bmx055data[5] * dt
		Motor.motor(mPL, mPR, dt, 1)
	else:
		Motor.motor(0, 0, 1)

		calData = Calibration.Calibration(fileCal)
		Other.saveLog(fileCal, ellipseScale)
		Other.saveLog(fileCal, time.time() - t_start)

	Motor.motor(0, 0, 1)

	print("Calibration Finished")
	return calData

def close():
	GPS.closeGPS()
	pi.write(22, 1)		#IM920 Turn On
	pi.write(17,0)
	Motor.motor(0, 0, 1)
	Motor.motor_stop()


if __name__ == "__main__":
	try:
		print("Program Start  {0}".format(time.time()))
		t_start = time.time()

		#-----setup phase ---------#
		setup()
		print("Start Phase is {0}".format(phaseChk))
		if(phaseChk <= 1):
			IM920.Send("P1S")
			Other.saveLog(phaseLog, "1", "Program Started", time.time() - t_start)
			IM920.Send("P1F")

		# ------------------- Sleep Phase --------------------- #
		if(phaseChk <= 2):
			Other.saveLog(phaseLog, "2", "Sleep Phase Started", time.time() - t_start)
			print("Sleep Phase Started  {0}".format(time.time() - t_start))
			IM920.Send("P2S")
			pi.write(22, 0)			#IM920 Turn Off
			print("IM920 Turn Off")
			t_sleep_start = time.time()

			# --- Sleep --- #
			while(time.time() - t_sleep_start <= t_sleep):
				Other.saveLog(sleepLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				takePhoto()
				time.sleep(1)
				#IM920.Send("P2D")
			#IM920.Send("P2F")

		# ------------------- Release Phase ------------------- #
		if(phaseChk <= 3):
			Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
			t_release_start = time.time()
			print("Releasing Phase Started  {0}".format(time.time() - t_start))
			#IM920.Send("P3S")

			# --- Release Judgement, "while" is for timeout --- #
			while (time.time() - t_release_start <= t_release):
				luxjudge,lcount = Release.luxdetect()
				pressjudge,acount = Release.pressdetect()
				t1 = time.time()
				if luxjudge == 1 or pressjudge == 1:
					Other.saveLog(releaseLog, time.time() - t_start, "Release Judged by Sensor", luxjudge, pressjudge)
					print("Rover has released")
					break
				elif luxjudge == 2 or pressjudge == 2:
					photoreleasejudge,fcount=Release.photoreleasedetect(photoName)
					if photoreleasejudge == 1:
						print("Rover has release by photojudge")
						break
					elif photoreleasejudge == 0:
						print("i2c is dead!")
				else:
					print("Rover is in rocket")
					#IM920.Send("P3D")
				print("l"+str(lcount)+"  a"+ str(acount)+"  f"+str(fcount))
				# --- Save Log and Take Photo --- #
				gpsData = GPS.readGPS()
				Other.saveLog(releaseLog, time.time() - t_start, acount, lcount, fcount, gpsData, TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				takePhoto()
				#IM920.Send("P3D")
			else:
				Other.saveLog(releaseLog, time.time() - t_start, "Release Judged by Timeout")
				print("Release Timeout")
			pi.write(22, 1)	#Turn on IM920
			time.sleep(1)
			IM920.Send("P3F")

		# ------------------- Landing Phase ------------------- #
		if(phaseChk <= 4):
			Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
			print("Landing Phase Started  {0}".format(time.time() - t_start))
			IM920.Send("P4S")
			t_land_start = time.time()

			# --- Landing Judgement, "while" is for timeout --- #
			while(time.time() - t_land_start <= t_land):
				pressjudge, pcount = Land.pressdetect()
				#gpsjudge, gacount = Land.gpsdetect()
				print(pcount)
				if pressjudge == 1: #and gpsdetect == 1:
					Other.saveLog(landingLog, time.time() - t_start, "Land Judged by Sensor", pressjudge, gpsjudge)
					print("Rover has Landed")
					break
				elif pressjudge == 0: #and gpsdetect == 0:
				    print("Descend now ")
				#elif pressjudge == 1 : #or gpsjudge == 1:
				#print("Landing JudgementNow")

				elif pressjudge == 2: #when bme is dead
					magnetlandjudge,mcount = Land.bmxdetect()
					if magnetlandjudge == 1:
						Other.saveLog(landingLog, time.time() - t_start, "Land Judged by BMX", pressjudge, magnetlandjudge)
						print("Rover has Emergency landed")
						break
					elif magnetlandjudge == 0:
						print("emergency Descend now")
				print("p"+str(pcount)+"  m"+str(mcount))
				# --- Save Log and Take Photo--- #
				for i in range(3):
					Other.saveLog(landingLog ,time.time() - t_start, pcount, gacount, mcount, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
					takePhoto()
				IM920.Send("P4D")
			else:
				Other.saveLog(landingLog, time.time() - t_start, "Land Judged by Timeout")
				print("Landing Timeout")
			IM920.Send("P4F")

		# ------------------- Melting Phase ------------------- #
		if(phaseChk <= 5):
			Other.saveLog(phaseLog,"5", "Melting Phase Started", time.time() - t_start)
			print("Melting Phase Started")
			IM920.Send("P5S")
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Start")
			for i in range(3):
				print("Melting " + str(i))
				Melting.Melting(t_melt)
				time.sleep(1)
				Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting" + str(i))
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Finished")
			IM920.Send("P5F")

		# ------------------- ParaAvoidance Phase ------------------- #
		if(phaseChk <= 6):
			Other.saveLog(phaseLog, "6", "ParaAvoidance Phase Started", time.time() - t_start)
			IM920.Send("P6S")

			print("ParaAvoidance Phase Started")
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Start")

			# --- Parachute Judge --- #
			print("START: Judge covered by Parachute")
			t_paraDete_start = time.time()
			while time.time() - t_paraDete_start < timeout_parachute:
				paraLuxflug, paraLux = ParaDetection.ParaJudge(LuxThd)
				Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), paraLuxflug, paraLux, LuxThd)
				if paraLuxflug == 1:
					break
			'''
			# --- StuckDetection --- #
			Motor.motor(15, 15, 0.9)
			Motor.motor(0, 0, 0.9)
			stuckFlug = stuckDetection.BMXstuckDetection(mp_max, stuckThd, PstuckCount, stuckCountThd)
			if stuckFlug == 1:
				Motor.motor(-70, 70, 1)
				Motor.motor(70, -70, 1)
				Motor.motor(-70, 70, 1)
				Motor.motor(70, -70, 1)
				Motor.motor(0, 0, 1)
			'''
			Motor.motor(15,15, 0.9)
			Motor.motor(0, 0, 0.9)
			t_paraDete_start = time.time()
			BMX055data = BMX055.bmx055_read()
			while BMX055data[2] < 5:
				if time.time() - t_paraDete_start > timeout_parachute:
					break
				Motor.motor(80, 80, 0.2, 1)
				Motor.motor(0, 0, 0.8)
				Motor.motor(-80, -80, 0.2, 1)
				Motor.motor(0, 0, 0.8)
				Motor.motor(15, 15, 0.9)
				Motor.motor(0, 0, 2)
				BMX055data = BMX055.bmx055_read()

			# --- Parachute Avoidance --- #
			print("START: Parachute avoidance")
			for i in range(2):	#Avoid Parachute two times
				Motor.motor(0, 0, 2)
				Motor.motor(15, 15, 0.9)
				Motor.motor(0, 0, 0.9)
				paraExsist, paraArea, photoName = ParaDetection.ParaDetection(photopath, H_min, H_max, S_thd)

				if paraExsist == 1:
					Motor.motor(-mp_max, -mp_max, 5)
					Motor.motor(0, 0, 1)
					Motor.motor(mp_max, mp_max, 0.5)
					Motor.motor(0 ,0, 1)
					Motor.motor(mp_max, mp_min, 1.0)
					Motor.motor(0, 0, 1)

				if paraExsist == 0:
					Motor.motor(mp_max, mp_max, 5)
					Motor.motor(0 ,0, 1)
					Motor.motor(-mp_max, -mp_max, 0.5)
					Motor.motor(0 ,0, 1)

				if paraExsist == -1:
					Motor.motor(mp_max, mp_max, 5)
					Motor.motor(0 ,0, 1)
					Motor.motor(-mp_max, -mp_max, 0.5)
					Motor.motor(0 ,0, 1)

				Other.saveLog(captureLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), photoName)
				Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), photoName, paraExsist, paraArea)
			Other.saveLog(paraAvoidanceLog, time.time() - t_start, GPS.readGPS(), "ParaAvoidance Finished")
			IM920.Send("P6F")

		# ------------------- Running Phase ------------------- #
		if(phaseChk <= 7):
			Other.saveLog(phaseLog, "7", "Running Phase Started", time.time() - t_start)
			print("Running Phase Started")
			IM920.Send("P7S")

			# --- Read GPS Data --- #
			print("Read GPS Data")
			while(not RunningGPS.checkGPSstatus(gpsData)):
				gpsData = GPS.readGPS()
				time.sleep(1)
			#stuckMode = Stuck.stuckDetection(gpsData[1], gpsData[2])

			t_calib_origin = time.time() - timeout_calibration - 20
			t_takePhoto_start = time.time()
			while(disGoal >= 5):
				# --- Get GPS Data --- #
				if(disGoal <= 15):
					kp = 0.9
					maxMP = 40
				else:
					kp = 0.3
					maxMP = 70

				if(RunningGPS.checkGPSstatus(gpsData)):
					nLat = gpsData[1]
					nLon = gpsData[2]
					print(nLat, nLon, disGoal, angGoal, nAng, rAng, mPL, mPR, mPS)
					IM920.Send("G" + str(nLat) + "	" + str(nLon))

				# --- Calibration --- #
				if(time.time() - t_calib_origin > timeout_calibration):
					IM920.Send("P7C")
					Motor.motor(0, 0, 2)

					# --- Send Photo --- #
					transmitPhoto()

					#Every [timeout_calibratoin] second,  Calibrate
					print("Calibration")
					ellipseScale = calibration()
					t_calib_origin = time.time()

				# --- Taking Photo and Check Stuck--- #
				if(time.time() - t_takePhoto_start > timeout_takePhoto):
					IM920.Send("P7T")
					Motor.motor(0, 0, 2)
					Motor.motor(30, 30, 0.5)
					Motor.motor(0, 0, 0.5)

					# --- Take Photo --- #
					takePhoto()

					# --- Read GPS Data --- #
					gpsData = GPS.readGPS()
					while(not RunningGPS.checkGPSstatus(gpsData)):
						gpsData = GPS.readGPS()
						time.sleep(1)

					# --- Check Stuck Mode --- #
					stuckMode = Stuck.stuckDetection(gpsData[1], gpsData[2])
					if not (stuckMode[0] == 0):
						Other.saveLog(stuckLog, time.time() - t_start, gpsData, stuckMode)
						if(stuckMode[0] == 2):
							# - Stuck -#
							if(stuckMode[1] <= 1):
								# - Stuck First Time - #
								print("Stuck" + str(stuckMode))
								Motor.motor(0, 0, 1)
							elif(stuckoMode[1] <= 3):
								# - Stuck a few Time - #
								print("Stuck" + str(stuckMode))
							else:
								# - Stuck Many Many Time - #
								print("Stuck" + str(stuckMode))								
								Motor.motor(-60, -60, 2)
								Motor.motor(0, 0, 1)
								Motor.motor(60, -60, 1)
								Motor.motor(0, 0, 1)
						elif(stuckMode[0] == 1):
							# - Roll Over - #
							print("Roll Overed")
					t_takePhoto_start = time.time()

				# --- Calculate disGoal and relAng and Motor Power --- #
				nAng = RunningGPS.calNAng(ellipseScale, angOffset)			#Calculate Rover Angle
				relAng[2] = relAng[1]
				relAng[1] = relAng[0]
				disGoal, angGoal, relAng[0] = RunningGPS.calGoal(nLat, nLon, gLat, gLon, nAng)
				rAng = np.median(relAng)									#Calculate angle between Rover and Goal
				mPL, mPR, mPS = RunningGPS.runMotorSpeed(rAng, kp, maxMP)	#Calculate Motor Power

				# --- Save Log --- #
				print(nLat, nLon, disGoal, angGoal, nAng, rAng, mPL, mPR, mPS)
				Other.saveLog(runningLog, time.time() - t_start, BMX055.bmx055_read(), nLat, nLon, disGoal, angGoal, nAng, rAng, mPL, mPR, mPS)
				gpsData = GPS.readGPS()
				Motor.motor(mPL, mPR, 0.05, 1)
			Motor.motor(0, 0, 1)
			print("Running Phase Finished")
			IM920.Send("P7F")

		# ------------------- GoalDetection Phase ------------------- #
		if(phaseChk <= 8):
			Other.saveLog(phaseLog, "8", "GoalDetection Phase Started", time.time() - t_start)
			print("Goal Detection Phase Started")
			IM920.Send("P8S")

			# --- Transmit Image --- #
			transmitPhoto()

			t_goalDete_start = time.time()
			t_stuckDete_start = time.time()
			while goalFlug != 0 or goalBufFlug != 0:
				if time.time() - t_goalDete_start > timeout_goalDete:
					break
				gpsdata = GPS.readGPS()
				goalBufFlug = goalFlug
				"""				
				# --- Stuck Detection --- #
				if time.time() - t_stuckDete_start > timeout_stuck:
					stuckFlug = stuckDetection.BMXstuckDetection(mp_max, stuckThd, stuckCount, stuckCountThd)
					if stuckFlug == 1:
						Motor.motor(-80, -80, 3, 1)
						Motor.motor(0 ,0, 2)
						Motor.motor(-80, 80, 3, 1)
						Motor.motor(0 ,0, 2)
						Motor.motor(80, 80, 3, 1)
						Motor.motor(0 ,0, 2)
						Motor.motor(80, -80, 3, 1)
						Motor.motor(0, 0, 2)
					t_stuckDete_start = time.time()
				"""
				# --- get information --- #
				Motor.motor(0, 0, 2)
				Motor.motor(15,15, 0.9)
				Motor.motor(0, 0, 1.0)
				goalFlug, goalArea, goalGAP, photoName = goal_detection.GoalDetection(photopath, H_min, H_max, S_thd, goalthd)
				print("flug", goalFlug, "area", goalArea, "GAP", goalGAP)
				#print("bomb",bomb)

				# --- goal --- #
				if goalFlug == 0:
					Motor.motor(30, 30 + mp_adj, 0.4)
					Motor.motor(0, 0, 0.8)

				# --- not detect --- #
				elif goalFlug == -1:
					if bomb == 1:
						Motor.motor(mp_max, mp_min + mp_adj, 0.4)
						Motor.motor(0, 0, 0.8)
						bomb = 1
					else:
						Motor.motor(mp_min, mp_max + mp_adj, 0.4)
						Motor.motor(0, 0, 0.8)
						bomb = 0

				# --- detect but no goal --- #
				elif goalFlug < 100:
					# --- target left --- #
					if goalArea < 3000 and goalArea > 0 and goalGAP < 0:
						MP = goal_detection.curvingSwitch(goalGAP, adj_add)
						Motor.motor(mp_max - MP, mp_max + mp_adj, 0.6)
						Motor.motor(0, 0, 0.8)
						bomb = 1
					# --- target right --- #
					elif goalArea < 3000 and goalArea > 0 and goalGAP >= 0:
						MP = goal_detection.curvingSwitch(goalGAP, adj_add)
						Motor.motor(mp_max, mp_max - MP + mp_adj, 0.6)
						Motor.motor(0, 0, 0.8)
						bomb = 0
					else:
						# --- near the target --- #
						if goalGAP < 0:
							MP = goal_detection.curvingSwitch(goalGAP, adj_add)
							Motor.motor(mp_min, mp_max + MP + mp_adj, 0.4)
							Motor.motor(0, 0, 0.8)
							bomb = 1
						else:
							MP = goal_detection.curvingSwitch(goalGAP, adj_add)
							Motor.motor(mp_max + MP, mp_min + mp_adj, 0.4)
							Motor.motor(0, 0, 0.8)
							bomb = 0
				elif goalFlug > 105:
					goalFlug = 0
					goalBufFlug = 0
				else:
					pass

				Other.saveLog(goalDetectionLog, time.time() - t_start, gpsData, goalFlug, goalArea, goalGAP, photoName)
				Other.saveLog(captureLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), photoName)
				IM920.Send("P8D")
			print("Goal Detection Phase Finished")
			IM920.Send("P8F")

		# ------------------- Sending Photo Phase ------------------- #
		if(phaseChk <= 9):
			Other.saveLog(phaseLog, "9", "Sending Photo Phase Started", time.time() - t_start)
			print("Sending Photo Phase Started")
			IM920.Send("P9S")
			for i in range(3):
				IM920.Send("P9D")
				transmitPhoto()
				time.sleep(1)
				IM920.Send("P9D")
			print("Sending Photo Phase Finished")
			IM920.Send("P9F")

		# ------------------- Program Finish ------------------- #
		print("Program Finished")
		IM920.Send("P10")
		Other.saveLog(phaseLog, "10", "Program Finished", time.time() - t_start)

		close()
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
		IM920.Send("KI")
	except:
		pi.write(17, 0)
		pi.write(22, 1)
		Motor.motor(0, 0, 1)
		print(traceback.format_exc())
		Other.saveLog(errorLog, time.time() - t_start, "Error")
		Other.saveLog(errorLog, traceback.format_exc())
		Other.saveLog(errorLog, "\n")
		IM920.Send("EO")
		#os.system('sudo reboot')
		close()
