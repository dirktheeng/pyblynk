# -*- coding: utf-8 -*-
"""
contains all functions and classes related to measuring roll and pitch for the rvp
"""
__author__ = 'Dirk Van Essendelft'
__date__ = '2016-07-09'
__version__ = '0.1.0'
__credits__ = 'Copyright, Dirk Van Essendelft'
__license__ = 'MIT'

from accel import AccellGyro as AG
from pythreadworker import PyThreadWorker

class PyTilt(PyThreadWorker):
	def __init__(self, currentPin=0, refVoltagePin=1, voltagePin=2, gain=2/3, aveFact=0.1):
		self.AG = AG()
		PyThreadWorker.__init__(self, self.measureTilt)

	def measureTilt(self, nSamples=1000, vect=None):
		self.tilt = self.AG.measureTilt(nSamples=nSamples, vect=vect)
