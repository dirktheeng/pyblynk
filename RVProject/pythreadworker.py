# -*- coding: utf-8 -*-
"""
basic threading class
"""

__author__ = 'Dirk Van Essendelft'
__date__ = '2016-07-09'
__version__ = '0.1.0'
__credits__ = 'Copyright, Dirk Van Essendelft'
__license__ = 'MIT'

import threading
import time

class PyThreadWorker(object):
	def __init__(self, function):
		self.function = function
		self.thread = None
		self.stop_thread = False

	def threadJob(self):
		'''
		worker for the threaded job
		'''
		while True:
			self.function()
			time.sleep(0.05)
			if self.stop_thread:
				break

	def startThread(self):
		'''
		starts the thread
		'''
		if not self.thread:
			self.thread = threading.Thread(name='RVPThread', target=self.threadJob)
			self.thread.setDaemon(True)
			self.thread.start()

	def stopThread(self):
		'''
		stops the thread
		'''
		self.stop_thread = True
		self.thread.join()