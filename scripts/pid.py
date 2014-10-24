import rospy
import logging
import math

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable
from cflib.crazyflie import Crazyflie
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import String

DEFAULT_PID_INTEGRATION_LIMIT = 5000.0

class PID:
	def __init__(self, reference=0.0, dt=20.0):
		self.reference = reference
		self.error = 0.0
		self.prev_error = 0.0
		self.dt = dt
		self.Kp = 0.0
		self.Ki = 0.0
		self.Kd = 0.0
		self.integral = 0.0
		self.derivative = 0.0
		self.int_limit_high = DEFAULT_PID_INTEGRATION_LIMIT
		self.int_limit_low = -1*DEFAULT_PID_INTEGRATION_LIMIT

	def setGains(self, p, i, d):
		self.Kp = p
		self.Ki = i
		self.Kd = d

	def setIntegratorLowLimit(self, limit):
		self.int_limit_low = limit

	def setIntegratorHighLimit(self, limit):
		self.int_limit_high = limit

	def reset(self):
		self.error = 0
		self.prev_error = 0
		self.integral = 0.0
		self.derivative = 0.0

	def setError(self, err):
		self.error = err

	def setReference(self, ref):
		self.reference = ref

	def getReference(self):
		return self.reference

	def setTimeStep(self, ts):
		self.dt = ts

	def update(self, measured, update_error):
		if (update_error):
			self.error = self.reference - measured

		self.integral += self.error * self.dt
		if (self.integral > self.int_limit_high):
			self.integral = self.int_limit_high
		elif (self.integral < self.int_limit_low):
			self.integral = self.int_limit_low

		self.derivative = (self.error - self.prev_error) / self.dt
		self.prev_error = self.error

		return -(self.Kp*self.error + self.Ki*self.integral + self.Kd*self.derivative)



