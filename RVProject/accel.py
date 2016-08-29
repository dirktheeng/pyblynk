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
		if not os.path.exists('./' + self.calFile):
			self.setCalibration()

	def measureTilt(self, nSamples=1000):
		meas = self._measureRaw(nSamples)
		roll = np.arctan(-meas[0]/meas[2])
		pitch = np.arctan(meas[1]/np.sqrt(np.power(meas[0],2)+np.power(meas[2],2)))
		return np.asarray([pitch, roll])

	def _measureRaw(self, nSamples=1000, norm=True):
		data = np.zeros((nSamples, 3))
		for i in range(nSamples):
			x = self.get_accel_data()
			data[i,:] = np.asarray([x['x'], x['y'], x['z']])
		if norm:
			meas = np.mean(data, 0)
		meas /= np.sqrt(np.sum(np.power(meas, 2)))

		return meas

	def setCalibration(self):
		while True:
			r = raw_input('On a flat, level surface, place sensor box upright, y to continue: ')
			if r == 'y':
				break
		a = self._measureRaw(500)
		while True:
			r = raw_input('On a flat, level surface, place sensor box on the right face, y to continue: ')
			if r == 'y':
				break
		b = self._measureRaw(500)
		while True:
			r = raw_input('On a flat, level surface, place sensor box on the front face, y to continue: ')
			if r == 'y':
				break
		c = self._measureRaw(500)

		calInList = [a, b, c]
		
		calInListOrder = [np.where(a == a.max())[0][0], np.where(b == b.max())[0][0], np.where(c == c.max())[0][0]]
		calInList = np.asarray([list(calInList[calInListOrder.index(i)]) for i in range(3)])
		print('callInList')
		print(calInList)
		self.mask = 1.0 - np.identity(3)
		print(self.mask)
		#self.calibObjective([0,0,0], (calInList))
		ret = minimize(self.calibObjective, [0,0,0], args=(calInList[0,:],calInList[1,:],calInList[2,:]))
		print(ret.x)
		print(ret.message)
		#print(np.dot(self.rotate3D(*ret.x), calInList))


	def rotateAboutAxis(self, axis, theta):
		return expm3(np.cross(np.eye(3), axis/norm(axis)*theta))

	def rotate3D(self, a, b, c):
		return np.dot(np.dot(self.rotateAboutAxis([1,0,0], a), self.rotateAboutAxis([0,1,0], b)), self.rotateAboutAxis([0,0,1], c))

	def calibObjective(self, x, *args):
		args = np.asarray(args)
		#print('args')
		#print(args)
		rm = self.rotate3D(*x)
		rot = np.dot(rm, args)
		#print(rot)
		mult = np.multiply(self.mask, rot)
		return np.sum(np.abs(mult))

if __name__ == '__main__':
	s = AccellGyro()
	s.measureTilt()
