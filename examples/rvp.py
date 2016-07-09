# -*- coding: utf-8 -*-
"""
	example hardware
"""
__author__	= """Dirk Van Essendelft"""
__date__ 		= "2016-07-09"
__version__	= "0.2.0"
__credits__	= """Copyright, Dirk Van Essendelft"""
__license__	= "MIT"


import sys
import os
import random
import time
sys.path.append(
	os.path.join(
		os.path.dirname(__file__),
		'..'
	)
)
	
TOKEN			= '9d25bd61aa1243819180b58e523a37b6'

#Imports
import lib.hw as blynk_hw
import lib.client as blynk_client
import Adafruit_ADS1x15

# Blynk Setup
class myHardware(blynk_hw.Hardware):
	"""
		you'll probably have to overload the On* calls,
		see lib/hw.py
	"""
	def OnVirtualRead(self, pin):
		print('OnVirdualRead', pin, type(pin))
		if pin == 0:
			# Battery Voltage
			return 14.28
		elif pin == 1:
			# Battery Current
			return random.random()
		elif pin == 2:
			# Inside Temp
			return 73.4
		elif pin == 3:
			# Inside Humidity
			return 20.1
		elif pin == 4:
			# Ouside Temp
			return 89.1
		elif pin == 5:
			# Outside Humidity
			return 30.0
		else:
			return 0

cConnection=blynk_client.TCP_Client('localhost')
if not cConnection.connect():
	print('Unable to connect')
	sys.exit(-1)

if not cConnection.auth(TOKEN):
	print('Unable to auth')
	
cHardware=myHardware(cConnection)

#ADS1115 setup
adc = Adafruit_ADS1x15.ADS1115()
# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 2/3
gainDict = {2/3:6.144, 1:4.096, 2:2.048, 4: 1.024, 8:0.512, 16:0.256}

#timers
timer1dt = 5
timer1 = time.time() + timer1dt

# Micro Controller Loop 
try:
	while True:
		cHardware.manage()
		currTime = time.time()
		if currTime > timer1:
			timer1 = currTime + timer1dt
			values = [0]*4
			for i in range(4):
				values[i] = adc.read_adc(i, gain=GAIN) / 32768.0 * gainDict[GAIN]
			print('| {0:6.4f} | {1:6.4f} | {2:6.4f} | {3:6.4f} |'.format(*values))
except KeyboardInterrupt:
	raise

