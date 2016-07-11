# -*- coding: utf-8 -*-
"""
	example hardware
"""
__author__	= """Dirk Van Essendelft"""
__date__ 		= "2016-07-09"
__version__	= "0.2.0"
__credits__	= """Copyright, Dirk Van Essendelft"""
__license__	= "MIT"

#Imports
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
	
import lib.hw as blynk_hw
import lib.client as blynk_client
from pypower import PyPower


TOKEN			= '9d25bd61aa1243819180b58e523a37b6'


#PyPower setup
pp = PyPower()
print('starting thread')
pp.startThread()
print('running...')

# Blynk Setup
class myHardware(blynk_hw.Hardware):
	"""
	RV project event handlers only need to define On* functions
	"""
	def OnVirtualRead(self, pin):
		print('OnVirdualRead', pin, type(pin))
		if pin == 0:
			# Battery Voltage
			return 14.28
		elif pin == 1:
			# Battery Current
			if pp.ave_current:
				return pp.ave_current
			else:
				return 0.0
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


#timers
timer1dt = 5
timer1 = time.time() + timer1dt

# Micro Controller Loop 
try:
	while True:
		cHardware.manage()		
except KeyboardInterrupt:
	print('Stopping Thread')
	pp.stopThread()
	print('Stopped')
	raise

