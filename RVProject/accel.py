from mpu6050 import mpu6050
import numpy as np
import os

sensor = mpu6050(0x68)

class AccellGyro(mpu6050):
	def __init__(self):
		mpu6050.__init__(self, 0x68)
		self.calFile = 'accelCal.txt'
		if not os.path.exists('./' + self.calFile):
			self.setCalibration()

	def measureTilt(self, nSamples=1000):
		data = np.zeros((nSamples, 3))
		for i in range(nSamples):
			x = self.get_accel_data()
			data[i,:] = np.asarray([x['x'], x['y'], x['z']])
		meas = np.mean(data, 0)
		roll = np.arctan(-meas[0]/meas[2])
		pitch = np.arctan(meas[1]/np.sqrt(np.power(meas[0],2)+np.power(meas[2],2)))
		return np.asarray([pitch, roll])

	def setCalibration(self):
		a = self.measureTilt(5000)
		np.savetxt(self.calFile, a)

if __name__ == '__main__':
	s = AccellGyro()
	s.measureTilt()
