#!/usr/bin/python
#
#	This program  reads the angles from the acceleromter, gyrscope
#	and mangnetometeron a BerryIMU connected to a Raspberry Pi.
#
#	This program includes two filters (low pass and mdeian) to improve the
#	values returned from BerryIMU by reducing noise.
#
#
#	http://ozzmaker.com/
#
#    Copyright (C) 2017  Mark Williams
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Library General Public
#    License as published by the Free Software Foundation; either
#    version 2 of the License, or (at your option) any later version.
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#    Library General Public License for more details.
#    You should have received a copy of the GNU Library General Public
#    License along with this library; if not, write to the Free
#    Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
#    MA 02111-1307, USA


import sys
import smbus
import time
import math
import Helper
import datetime

from LSM9DS0 import *
from Helper import *

bus = smbus.SMBus(1)

def writeMAG(CTRL_REG7_XM, param):
	pass

log = LogHelper()

class IMU:

	log.logInfo('IMU data initiated...')

	IMU_upside_down = 0 	# Change calculations depending on IMu orientation.
							# 0 = Correct side up. This is when the skull logo is facing down
							# 1 = Upside down. This is when the skull logo is facing up

	RAD_TO_DEG = 57.29578
	M_PI = 3.14159265358979323846
	G_GAIN = 0.070  	# [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
	AA =  0.40      	# Complementary filter constant
	MAG_LPF_FACTOR = 0.4 	# Low pass filter constant magnetometer
	ACC_LPF_FACTOR = 0.4 	# Low pass filter constant for accelerometer
	ACC_MEDIANTABLESIZE = 9    	# Median filter table size for accelerometer. Higher = smoother but a longer delay
	MAG_MEDIANTABLESIZE = 9    	# Median filter table size for magnetometer. Higher = smoother but a longer delay

	#Kalman filter variables
	Q_angle = 0.02
	Q_gyro = 0.0015
	R_angle = 0.005
	y_bias = 0.0
	x_bias = 0.0
	XP_00 = 0.0
	XP_01 = 0.0
	XP_10 = 0.0
	XP_11 = 0.0
	YP_00 = 0.0
	YP_01 = 0.0
	YP_10 = 0.0
	YP_11 = 0.0
	KFangleX = 0.0
	KFangleY = 0.0

	READ_IMU_DATA = 0

	DATA = list()

	def kalmanFilterY (self,accAngle, gyroRate, DT):
		y=0.0
		S=0.0

		#global KFangleY
		#global Q_angle
		#global Q_gyro
		#global y_bias
		#global YP_00
		#global YP_01
		#global YP_10
		#global YP_11

		self.KFangleY = self.KFangleY + DT * (gyroRate - self.y_bias)

		self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) +  self.Q_angle * DT )
		self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
		self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
		self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )

		y = accAngle - self.KFangleY
		S = self.YP_00 +  self.R_angle
		K_0 = self.YP_00 / S
		K_1 = self.YP_10 / S

		KFangleY = self.KFangleY + ( K_0 * y )
		self.y_bias = self.y_bias + ( K_1 * y )

		self.YP_00 = self.YP_00 - ( K_0 * self.YP_00 )
		self.YP_01 = self.YP_01 - ( K_0 * self.YP_01 )
		self.YP_10 = self.YP_10 - ( K_1 * self.YP_00 )
		self.YP_11 = self.YP_11 - ( K_1 * self.YP_01 )

		return KFangleY

	def kalmanFilterX (self, accAngle, gyroRate, DT):
		x=0.0
		S=0.0

		#global KFangleX
		#global Q_angle
		#global Q_gyro
		#global x_bias
		#global XP_00
		#global XP_01
		#global XP_10
		#global XP_11


		self.KFangleX = self.KFangleX + DT * (gyroRate - self.x_bias)

		self.XP_00 = self.XP_00 + ( - DT * (self.XP_10 + self.XP_01) +  self.Q_angle * DT )
		self.XP_01 = self.XP_01 + ( - DT * self.XP_11 )
		self.XP_10 = self.XP_10 + ( - DT * self.XP_11 )
		self.XP_11 = self.XP_11 + ( +  self.Q_gyro * DT )

		x = accAngle - self.KFangleX
		S = self.XP_00 +  self.R_angle
		K_0 = self.XP_00 / S
		K_1 = self.XP_10 / S

		self.KFangleX = self.KFangleX + ( K_0 * x )
		x_bias = self.x_bias + ( K_1 * x )

		self.XP_00 = self.XP_00 - ( K_0 * self.XP_00 )
		self.XP_01 = self.XP_01 - ( K_0 * self.XP_01 )
		self.XP_10 = self.XP_10 - ( K_1 * self.XP_00 )
		self.XP_11 = self.XP_11 - ( K_1 * self.XP_01 )

		return self.KFangleX

	def writeACC(self,register,value):
		bus.write_byte_data(ACC_ADDRESS , register, value)
		return -1

	def writeMAG(self,register,value):
		bus.write_byte_data(MAG_ADDRESS, register, value)
		return -1

	def writeGRY(self,register,value):
		bus.write_byte_data(GYR_ADDRESS, register, value)
		return -1

	def readACCx(self):
		acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
		acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
		acc_combined = (acc_l | acc_h <<8)

		return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def readACCy(self):
		acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
		acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
		acc_combined = (acc_l | acc_h <<8)

		return acc_combined  if acc_combined < 32768 else acc_combined - 65536


	def readACCz(self):
		acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
		acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
		acc_combined = (acc_l | acc_h <<8)
		return acc_combined  if acc_combined < 32768 else acc_combined - 65536


	def readMAGx(self):
		mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
		mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
		mag_combined = (mag_l | mag_h <<8)

		return mag_combined  if mag_combined < 32768 else mag_combined - 65536


	def readMAGy(self):
		mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
		mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
		mag_combined = (mag_l | mag_h <<8)

		return mag_combined  if mag_combined < 32768 else mag_combined - 65536


	def readMAGz(self):
		mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
		mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
		mag_combined = (mag_l | mag_h <<8)

		return mag_combined  if mag_combined < 32768 else mag_combined - 65536



	def readGYRx(self):
		gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
		gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
		gyr_combined = (gyr_l | gyr_h <<8)

		return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def readGYRy(self):
		gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
		gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
		gyr_combined = (gyr_l | gyr_h <<8)

		return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def readGYRz(self):
		gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
		gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
		gyr_combined = (gyr_l | gyr_h <<8)

		return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def __init__(self):

		log.logInfo('Setting up IMU configuration....')

		self.writeACC(CTRL_REG1_XM, 0b01100111)
		self.writeACC(CTRL_REG2_XM, 0b00100000)

		#initialise the magnetometer
		self.writeMAG(CTRL_REG5_XM, 0b11110000)
		self.writeMAG(CTRL_REG6_XM, 0b01100000)
		self.writeMAG(CTRL_REG7_XM, 0b00000000)

		#initialise the gyroscope
		self.writeGRY(CTRL_REG1_G, 0b00001111)
		self.writeGRY(CTRL_REG4_G, 0b00110000)


		self.gyroXangle = 0.0
		self.gyroYangle = 0.0
		self.gyroZangle = 0.0
		self.CFangleX = 0.0
		self.CFangleY = 0.0
		self.CFangleXFiltered = 0.0
		self.CFangleYFiltered = 0.0
		self.kalmanX = 0.0
		self.kalmanY = 0.0
		self.oldXMagRawValue = 0
		self.oldYMagRawValue = 0
		self.oldZMagRawValue = 0
		self.oldXAccRawValue = 0
		self.oldYAccRawValue = 0
		self.oldZAccRawValue = 0
		
		self.acc_medianTable1X = [1] * self.ACC_MEDIANTABLESIZE
		self.acc_medianTable1Y = [1] * self.ACC_MEDIANTABLESIZE
		self.acc_medianTable1Z = [1] * self.ACC_MEDIANTABLESIZE
		self.acc_medianTable2X = [1] * self.ACC_MEDIANTABLESIZE
		self.acc_medianTable2Y = [1] * self.ACC_MEDIANTABLESIZE
		self.acc_medianTable2Z = [1] * self.ACC_MEDIANTABLESIZE
		self.mag_medianTable1X = [1] * self.MAG_MEDIANTABLESIZE
		self.mag_medianTable1Y = [1] * self.MAG_MEDIANTABLESIZE
		self.mag_medianTable1Z = [1] * self.MAG_MEDIANTABLESIZE
		self.mag_medianTable2X = [1] * self.MAG_MEDIANTABLESIZE
		self.mag_medianTable2Y = [1] * self.MAG_MEDIANTABLESIZE
		self.mag_medianTable2Z = [1] * self.MAG_MEDIANTABLESIZE

		

	#def read(self):
		#print('IMU function')

	def read(self):
		log.logInfo('Data reading initiated')
		a = datetime.datetime.now()
		counter = 0;
		while self.READ_IMU_DATA == 1:
			
			row = []
			#Read the accelerometer,gyroscope and magnetometer values
			ACCx = self.readACCx()
			ACCy = self.readACCy()
			ACCz = self.readACCz()
			GYRx = self.readGYRx()
			GYRy = self.readGYRy()
			GYRz = self.readGYRz()
			MAGx = self.readMAGx()
			MAGy = self.readMAGy()
			MAGz = self.readMAGz()

			



			###############################################
			#### Apply low pass filter ####
			###############################################
			MAGx =  MAGx  * self.MAG_LPF_FACTOR + self.oldXMagRawValue*(1 - self.MAG_LPF_FACTOR);
			MAGy =  MAGy  * self.MAG_LPF_FACTOR + self.oldYMagRawValue*(1 - self.MAG_LPF_FACTOR);
			MAGz =  MAGz  * self.MAG_LPF_FACTOR + self.oldZMagRawValue*(1 - self.MAG_LPF_FACTOR);
			ACCx =  ACCx  * self.ACC_LPF_FACTOR + self.oldXAccRawValue*(1 - self.ACC_LPF_FACTOR);
			ACCy =  ACCy  * self.ACC_LPF_FACTOR + self.oldYAccRawValue*(1 - self.ACC_LPF_FACTOR);
			ACCz =  ACCz  * self.ACC_LPF_FACTOR + self.oldZAccRawValue*(1 - self.ACC_LPF_FACTOR);

			self.oldXMagRawValue = MAGx
			self.oldYMagRawValue = MAGy
			self.oldZMagRawValue = MAGz
			self.oldXAccRawValue = ACCx
			self.oldYAccRawValue = ACCy
			self.oldZAccRawValue = ACCz




			#########################################
			#### Median filter for accelerometer ####
			#########################################
			# cycle the table
			for x in range (self.ACC_MEDIANTABLESIZE-1,0,-1 ):
				self.acc_medianTable1X[x] = self.acc_medianTable1X[x-1]
				self.acc_medianTable1Y[x] = self.acc_medianTable1Y[x-1]
				self.acc_medianTable1Z[x] = self.acc_medianTable1Z[x-1]

			# Insert the lates values
			self.acc_medianTable1X[0] = ACCx
			self.acc_medianTable1Y[0] = ACCy
			self.acc_medianTable1Z[0] = ACCz

			# Copy the tables
			acc_medianTable2X = self.acc_medianTable1X[:]
			acc_medianTable2Y = self.acc_medianTable1Y[:]
			acc_medianTable2Z = self.acc_medianTable1Z[:]

			# Sort table 2
			acc_medianTable2X.sort()
			acc_medianTable2Y.sort()
			acc_medianTable2Z.sort()

			# The middle value is the value we are interested in
			ACCx = acc_medianTable2X[self.ACC_MEDIANTABLESIZE/2];
			ACCy = acc_medianTable2Y[self.ACC_MEDIANTABLESIZE/2];
			ACCz = acc_medianTable2Z[self.ACC_MEDIANTABLESIZE/2];




			#########################################
			#### Median filter for magnetometer ####
			#########################################
			# cycle the table
			for x in range (self.MAG_MEDIANTABLESIZE-1,0,-1 ):
				self.mag_medianTable1X[x] = self.mag_medianTable1X[x-1]
				self.mag_medianTable1Y[x] = self.mag_medianTable1Y[x-1]
				self.mag_medianTable1Z[x] = self.mag_medianTable1Z[x-1]

			# Insert the lates values
			self.mag_medianTable1X[0] = MAGx
			self.mag_medianTable1Y[0] = MAGy
			self.mag_medianTable1Z[0] = MAGz

			# Copy the tables
			mag_medianTable2X = self.mag_medianTable1X[:]
			mag_medianTable2Y = self.mag_medianTable1Y[:]
			mag_medianTable2Z = self.mag_medianTable1Z[:]

			# Sort table 2
			mag_medianTable2X.sort()
			mag_medianTable2Y.sort()
			mag_medianTable2Z.sort()

			# The middle value is the value we are interested in
			MAGx = mag_medianTable2X[self.MAG_MEDIANTABLESIZE/2];
			MAGy = mag_medianTable2Y[self.MAG_MEDIANTABLESIZE/2];
			MAGz = mag_medianTable2Z[self.MAG_MEDIANTABLESIZE/2];

			#Discard initial readings
			if counter < 10 :
				counter = counter + 1
				continue;

			##Calculate loop Period(LP). How long between Gyro Reads
			b = datetime.datetime.now() - a
			a = datetime.datetime.now()
			LP = b.microseconds/(1000000*1.0)
			print "Loop Time | %5.2f|" % ( LP )

			row.append(a)
			row.append(ACCx)
			row.append(ACCy)
			row.append(ACCz)
			row.append(GYRx)
			row.append(GYRy)
			row.append(GYRz)
			row.append(MAGx)
			row.append(MAGy)
			row.append(MAGz)

			self.DATA.append(row)

			if 1:			#Change to '0' to stop  showing the angles from the gyro
				print ("\033[1;31;40m\tTimestamp %s " % (str(a))),

			if 1:			#Change to '0' to stop showing the angles from the accelerometer
				print ("\033[1;34;40mACCX %5.2f ACCY %5.2f  ACCZ %5.2f \033[0m  " % (ACCx, ACCy,ACCz)),

			if 1:			#Change to '0' to stop  showing the angles from the gyro
				print ("\033[1;33;40m\tGRYX %5.2f  GYRY %5.2f  GYRZ %5.2f" % (GYRx,GYRy,GYRz)),

			if 1:			#Change to '0' to stop  showing the angles from the complementary filter
				print ("\033[1;35;40m\tMAGx %5.2f  MAGy %5.2f  MAGz %5.2f \33[1;32;40m" % (MAGx,MAGy,MAGz))

			#slow program down a bit, makes the output more readable
			time.sleep(0.01)

	#print('End of read')
