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
from scipy.linalg import expm3, norm
import os

sensor = mpu6050(0x68)

class AccellGyro(mpu6050):
	def __init__(self):
		mpu6050.__init__(self, 0x68)
		self.calFile = 'accelCal.txt'
		self.angCorFile = 'angCor.txt'
		self.oSetFile = 'oSet.txt'
		if not os.path.exists('./' + self.oSetFile):
			self.setCalibration()
		else:
			self.oSet = np.loadtxt('./'+self.oSetFile)


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
		print('vect')
		print(vect)
		meas = vect
		pitch = self._calcPitch(vect)
		roll = self._calcRoll(vect)
		return np.asarray([pitch, roll])


	def _measureRaw(self, nSamples=1000, normalize=True):
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
		meas = np.mean(data, 0)
		if normalize:
			meas /= norm(meas)

		return meas

	def _calcPitch(self, v):
		c = self.projectOntoPlane(v, self.oSet[1, :])
		return self.angleBtVectors(self.oSet[2, :], c) - np.pi/2.0

	def _calcRoll(self, v):
		c = self.projectOntoPlane(v, self.oSet[2, :])
		return self.angleBtVectors(self.oSet[1, :], c) - np.pi/2.0

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

		c = np.cross(a, b)
		b = np.cross(a, c)
		a /= norm(a)
		b /= norm(b)
		c /= norm(c)
		oSet = np.zeros((3,3))
		oSet[0, :] = a
		oSet[1, :] = b
		oSet[2, :] = c
		np.savetxt('./' + self.oSetFile, oSet)
		self.oSet = oSet

	def projectOntoPlane(self, vect, normal):
		nn = norm(normal)
		d = np.dot(vect, normal) / nn
		p = np.multiply(d, normal / nn)
		return vect - p

	def angleBtVectors(self, v1, v2):
		v1 /= norm(v1)
		v2 /= norm(v2)
		return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

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

if __name__ == '__main__':
	s = AccellGyro()
	print('roll and pitch:')
	print(s.measureTilt()*180.0/np.pi)
