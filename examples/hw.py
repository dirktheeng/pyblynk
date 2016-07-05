# -*- coding: utf-8 -*-
"""
	example hardware
"""
__author__	= """Alexander Krause <alexander.krause@ed-solutions.de>"""
__date__ 		= "2016-01-11"
__version__	= "0.2.0"
__credits__	= """Copyright e-design, Alexander Krause <alexander.krause@ed-solutions.de>"""
__license__	= "MIT"


import sys
import os
import random
sys.path.append(
	os.path.join(
		os.path.dirname(__file__),
		'..'
	)
)
	
TOKEN			= '9d25bd61aa1243819180b58e523a37b6'

import lib.hw as blynk_hw
import lib.client as blynk_client

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

try:
	while True:
		cHardware.manage()
except KeyboardInterrupt:
	raise

