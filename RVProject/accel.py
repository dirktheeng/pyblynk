# -*- coding: utf-8 -*-
"""
a library for measuring roll and pitch from an accelerameter inside a project box.
The library contains algorithms to account for misalignment of the sensor with the projecr box.
"""

__author__ = 'Dirk Van Essendelft'
__date__ = '2016-07-09'
__version__ = '0.1.0'
__credits__ = 'Copyright, Dirk Van Essendelft'
__license__ = 'MIT'

from mpu6050 import mpu6050
import numpy as np
from scipy.optimize import minimize
from scipy.linalg import expm3, norm
import os

sensor = mpu6050(0x68)

class AccellGyro(mpu6050):
	def __init__(self):
		mpu6050.__init__(self, 0x68)
		self.calFile = 'accelCal.txt'
		self.angCorFile = 'angCor.txt'
		if not os.path.exists('./' + self.calFile) or not os.path.exists('./'+self.angCorFile):
			self.angCor = np.asarray([0,0])
			self.setCalibration()
		else:
			self.rotationMatrix = np.loadtxt('./'+self.calFile)
			self.angCor = np.loadtxt('./'+self.angCorFile)

	def measureTilt(self, nSamples=1000, vect=None):
		'''
		Claculates the roll ans pitch angles after being calibrated
		
		Inputs
		------
		nSamples (int):
			The number of sample measueements to make to calculate the roll and pitch

		vect (array):
			A unit vector substitute for measurements
		
		Retuns
		------
		rp (array):
			a 2 component vector of the roll pitchdata  

		'''
		if vect is None:
			vect = self._measureRaw(nSamples)
		raw = self._measureRaw(nSamples)
		meas = np.dot(self.rotationMatrix, vect)
		roll = np.arctan(-meas[0]/meas[2])
		pitch = np.arctan(meas[1]/np.sqrt(np.power(meas[0],2)+np.power(meas[2],2)))
		return np.asarray([pitch, roll])+self.angCor

	def _measureRaw(self, nSamples=1000, norm=True):
		'''
		Claculates the vector from the acceleeameter
		
		Inputs
		------
		nSamples (int):
			The number of sample measueements to make to calculate the vector
		
		norm (bool):
			A flag that sets normalization toma unit vector

		Retuns
		------
		meas (array):
			a 3 component vector of the accelerameter data  
		'''
		data = np.zeros((nSamples, 3))
		for i in range(nSamples):
			x = self.get_accel_data()
			data[i,:] = np.asarray([x['x'], x['y'], x['z']])
		if norm:
			meas = np.mean(data, 0)
		meas /= np.sqrt(np.sum(np.power(meas, 2)))

		return meas

	def setCalibration(self):
		'''
		calculates the necessary calibratiin parameters to account for sensor misalignment relative to the project box
		'''
		print('Calibration file not found. Starting Calibration Proceedure.')
		while True:
			r = raw_input('On a flat, level surface, place sensor box upright, y to continue: ')
			if r == 'y':
				break
		a = self._measureRaw(5000)
		while True:
			r = raw_input('On a flat, level surface, place sensor box on the right face, y to continue: ')
			if r == 'y':
				break
		b = self._measureRaw(5000)
		while True:
			r = raw_input('On a flat, level surface, place sensor box on the front face, y to continue: ')
			if r == 'y':
				break
		c = self._measureRaw(5000)

		calInList = [a, b, c]
		
		calInListOrder = [np.where(a == a.max())[0][0], np.where(b == b.max())[0][0], np.where(c == c.max())[0][0]]
		calInList = np.asarray([list(calInList[calInListOrder.index(i)]) for i in range(3)])
		self.mask = 1.0 - np.identity(3)
		print
		print('Measurements complete, starting optimization....')
		ret = minimize(self.calibObjective, [0,0,0], args=(calInList[0,:],calInList[1,:],calInList[2,:]), tol=1e-10)
		print(ret.message)
		print
		print('Optimized Calibration Angles (deg):')
		ang = ret.x
		print(ang*180.0/(np.pi))
		print
		self.rotationMatrix = self.rotate3D(*ang)
		print('Calibration complete! Final Rotation Matrix:')
		print(self.rotationMatrix)
		print
		self.angCor = -1.0*self.measureTilt(vect=a)
		print('Angle Correction (deg):')
		print(self.angCor*180.0/np.pi)
		print
		np.savetxt('./'+self.calFile, self.rotationMatrix)
		np.savetxt('./'+self.angCorFile, self.angCor)


	def rotateAboutAxis(self, axis, theta):
		'''
		rotates a vector about a given axis a given ammount

		Inputs
		------
		axis (array):
			vector defining rotation axis

		theta (float):
			angle in radians to rotate

		Returns
		-------
		vNew (array):
			rotated vector
		'''
		return expm3(np.cross(np.eye(3), axis/norm(axis)*theta))

	def rotate3D(self, a, b, c):
		'''
		combined rotation around the 3 cartesian axes

		Inputs
		------
		a, b, c (float):
			angle in radians to rotate about axes

		Returns
		-------
		vNew (array):
			rotated vector
		'''
		return np.dot(np.dot(self.rotateAboutAxis([1,0,0], a), self.rotateAboutAxis([0,1,0], b)), self.rotateAboutAxis([0,0,1], c))

	def calibObjective(self, x, *args):
		'''
		objective function for solving rotation matrix values
		'''
		args = np.asarray(args)
		rm = self.rotate3D(*x)
		rot = np.dot(rm, args)
		mult = np.multiply(self.mask, rot)
		err1 = np.sum(np.abs(mult))
		return -1.0*(rot[0,0]+rot[1,1]+rot[2,2]) + err1

if __name__ == '__main__':
	s = AccellGyro()
	print('roll and pitch:')
	print(s.measureTilt()*180.0/np.pi)
