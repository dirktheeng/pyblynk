# -*- coding: utf-8 -*-
"""
contains all functions and classes related to measuring power for the rvp
"""
import Adafruit_ADS1x15
import threading
import time

class PyPower(object):
	def __init__(self, currentPin=0, refVoltagePin=1, voltagePin=2, gain=2/3, aveFact=0.1):
		self.current_pin = currentPin
		self.ref_voltage_pin = refVoltagePin
		self.voltage_pin = voltagePin
		self.ave_fact = aveFact
		self._adc = Adafruit_ADS1x15.ADS1115()
		self.gain = gain
		self.gain_dict = {2/3:6.144, 1:4.096, 2:2.048, 4: 1.024, 8:0.512, 16:0.256}
		self.curr_slope = (0.058 - 0.024)/(12.0 - 5.0)
		self.curr_intercept = 0.058 - 12.0*self.curr_slope
		self.ave_current = None
		self.thread = None
		self.stop_thread = False

	def _measureVoltage(self, pin):
		return self._adc.read_adc(pin, gain=self.gain) / 32768.0 * self.gain_dict[self.gain]

	def measureCurrent(self):
		curr_voltage = self._measureVoltage(self.current_pin)
		ref_voltage  = self._measureVoltage(self.ref_voltage_pin)
		sensitivity = self._calcSensitivity(ref_voltage)
		current = (curr_voltage - ref_voltage/2.0) * sensitivity
		if self.ave_current:
			self.ave_current = self.ave_fact * current + (1.0 - self.ave_fact) * self.ave_current
		else:
			self.ave_current = current
#		print curr_voltage, ref_voltage, sensitivity, self.ave_current

	def _calcSensitivity(self, voltage):
		return 1.0/(self.curr_slope * voltage + self.curr_intercept)

	def threadJob(self):
		while True:
			self.measureCurrent()
			time.sleep(0.05)
			if self.stop_thread:
				break

	def startThread(self):
		if not self.thread:
			self.thread = threading.Thread(name='RVPThread', target=self.threadJob)
			self.thread.setDaemon(True)
			self.thread.start()

	def stopThread(self):
		self.stop_thread = True
		self.thread.join()
