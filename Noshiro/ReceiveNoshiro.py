import sys
import binascii
import codecs
import cv2
import datetime
import linecache
import matplotlib.pyplot as plt
import numpy as np
import os
import serial
import time
import traceback
import warnings
from PIL import Image
from matplotlib import pyplot as plt

baudrate = 57600
comNum = 'COM37'
com = 0
receptionLog = r"communicationLog.txt"
receptionDecrptionLog = r"communicationDecryptionLog.txt"
receivePhotoArrayLogPath = r"receivePhotoLog"
receivePhotoArrayLog = ""
array = [[[0]*3]*80]*60
array = np.zeros_like(array)
I = list(range(5))
receivePhotFlug = 0
receivePhotoPath = "receivePhoto"
restorePhotoPath = "restorePhoto"
receivePhotoName = ""
restorePhotoName = ""
receivePhotoData = ['', '', '', '', '']
receivePhotoNum = [0, 0, 0, 0, 0]

def setSerial(mybaudrate=19200):
    com = serial.Serial(
        port=comNum,
        baudrate=mybaudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        timeout=3,
        xonxoff=False,
        rtscts=False,
        writeTimeout=None,
        dsrdtr=False,
        interCharTimeout=None)
    com.flushInput()
    com.flushOutput()
    return com

def Reception(mybaudrate =19200):
    global com
    try:
        com.flushInput()
        cngtext = ""
        text = com.readline().decode('utf-8').strip()
        com.flushOutput()
        text = text.replace("\r\n","")
        textData = text.split(":")[1]
        textData = textData.split(",")
        for x in textData:
            cngtext += chr(int(x,16))
    except Exception:
        cngtext = ""
        #print("NO DATA")
    return text, cngtext

def saveLog(path, *data):
	with open(path, "a") as f:
		for i in range(len(data)):
			if isinstance(data[i], list):
				for j in range(len(data[i])):
					f.write(str(data[i][j]) + "\t")
					#print(str(data[i][j]) + "\t", end="")
			else:
				f.write(str(data[i]) + "\t")
		f.write("\n")
		#print()

def fileName(f, ext):
	i = 0
	while(os.path.exists(f+str(i) + "." + ext)):
		i = i + 1
	f = f + str(i) + "." + ext
	return f

if __name__ == "__main__":
    try:
        count = 0
        com = setSerial(baudrate)
        while 1:
            receiveData, receiveDataDec = Reception(baudrate)
            if(receiveDataDec == ""):
                continue
            elif(65 <= ord(receiveDataDec[0]) <= 90):
                # --- Large Alphabet --- #
                saveLog(receptionLog, datetime.datetime.now(), receiveData)
                saveLog(receptionDecrptionLog, datetime.datetime.now(), receiveDataDec)
                if(receivePhotoFlug == 1):
                    receivePhotoName = fileName(receivePhotoPath, 'jpg')
                    cv2.imwrite(receivePhotoName, array)
                    receivePhotoArrayLog = fileName(receivePhotoArrayLogPath, 'txt')
                    receivePhotoFlug = 0
                    for i in range(len(array)):
                        for j in range(len(array[i])):
                            if np.allclose(array[i][j], [0, 0, 0]):
                                for k in range(len(array[i][j])):
                                    pixel = 0
                                    num = 0
                                    if i != 0:
                                        pixel = array[i-1][j][k]
                                        num = num + 1
                                    if j != 0:
                                        pixel = pixel + array[i][j-1][k]
                                        num = num + 1
                                    if i != 59:
                                        pixel = pixel + array[i+1][j][k]
                                        num = num + 1
                                    if j != 79:
                                        pixel = pixel + array[i][j+1][k]
                                        num = num + 1
                                    array[i][j][k] = int(pixel / num)                    
                    restorePhotoName = fileName(restorePhotoPath, 'jpg')
                    cv2.imwrite(restorePhotoName, array)
                    print(count)
                    array = np.zeros_like(array)
            elif(48 <= ord(receiveDataDec[0]) <= 57):
                # --- Number --- #
                receivePhotoFlug = 1
                receivePhotoData[4] = receiveDataDec[-3:]
                receivePhotoData[3] = receiveDataDec[-6:-3]
                receivePhotoData[2] = receiveDataDec[-9:-6]
                receivePhotoData[1] = receiveDataDec[-12:-9]
                receivePhotoData[0] = receiveDataDec[:-12]

                for i in range(len(receivePhotoData)):
                    if receivePhotoData[i] == '':
                        receivePhotoData[i] = '0'
                    receivePhotoNum[i] = int(receivePhotoData[i])

                print(receivePhotoNum)
            
                for i in range(3):
                    array[receivePhotoNum[0]][receivePhotoNum[1]][i] = receivePhotoNum[i+2]
    except:
        com.close()
        print(traceback.format_exc())