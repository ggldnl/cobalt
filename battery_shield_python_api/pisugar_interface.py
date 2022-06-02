import abc # Abstract Base Classes to implement formal interfaces

# threading to constantly get an averaged value for
# voltage and current draw
from threading import Thread
import time

# datetime to get a timestamp for each measure
from datetime import datetime

# this is how things should go (background thread doing updates), 
# but the result is really disappointing because the updates do 
# not respect the set times. I think the problem is python 
# (GIL + slowness of the language) because threads in c++ did not show
# these problems (despite being the RPi Zero single core)
class PiSugarInterface(abc.ABC, Thread):
	"""
	This interface is used for concrete classes to inherit from.
	Methods defined here will be inherited by the subclasses.
	We can define here the standard behavior common between all
	PiSugar versions and describe in particular each version
	(registers etc.) in the subclasses
	"""

	def __init__ (self):

		# I2C info
		self.I2C_BUS = None
		self.I2C_ADDRESS  = None
		self.I2C_CMD_VH   = None # voltage high byte
		self.I2C_CMD_VL   = None # voltage low byte
		self.I2C_CMD_IH   = None # current high byte
		self.I2C_CMD_IL   = None # current low byte
		self.I2C_CMD_TEMP = None # temperature
		self.I2C_CMD_CTR1 = None # global ctrl 1
		self.I2C_CMD_CTR2 = None # global ctrl 2

		# with SMBus(I2C_BUS) as bus: remember to close it
		self._bus = None

		self._model = None

		self._battery_curve = None


		# each observation is a tuple (time, observation)
		self.HISTORY_LEN = 30 # how many voltage/current observations should we keep
		self.UPDATE_INTERVAL = 1 # update each n(=1) seconds

		# array to store voltage data over time (need for avg)
		self._voltages = [(0.0, None) for i in range(self.HISTORY_LEN)]
		self._avg_voltage = 0.0
		self._avg_percent = 0.0

		# array to store current draw data over time (need for avg)
		self._current_draw = [(0.0, None) for i in range(self.HISTORY_LEN)]
		self._avg_output_current = True

		self.should_run = True
		self.polling_index = 0 # index in which to put the measurement (voltage or current)

		# The array will fill up slowly. Computing the mean we need to
		# account only for the values that we added
		self.added_elements = 0


	def run (self):

		# we won't need locks and atomic blocks because of the GIL 
		# (Global Interpreter Lock) (?)
		# It guarantees no two threads are executing Python code at the same time, 
		# so there can never be simultaneous reads/writes.
		while self.should_run:

			if self.added_elements < self.HISTORY_LEN:
				self.added_elements += 1 # max = HISTORY_LEN

			# get voltage
			v = self.voltage()
			self._voltages[self.polling_index] = (v, datetime.now())

			# get current
			i = self.output_current()
			self._current_draw[self.polling_index] = (i, datetime.now())

			# update polling_index
			self.polling_index = (self.polling_index + 1) % self.HISTORY_LEN

			# update mean values
			self._avg_voltage = 0.0
			self._avg_output_current = 0.0

			for i in range(self.added_elements):
				self._avg_voltage += self._voltages[i][0]
				self._avg_output_current += self._current_draw[i][0]

			self._avg_voltage /= self.added_elements
			self._avg_output_current /= self.added_elements

			# update average percentage
			self._avg_percent = 100.0
			if self._avg_voltage > 0.0:
				self._avg_percent = self._convert_battery_voltage_to_level(self._avg_voltage, self._battery_curve)

			# sleep
			time.sleep(self.UPDATE_INTERVAL)


	def stop (self):
		self.should_run = False


	def __exit__(self, exc_type, exc_value, traceback):
		'''
		Close the stream and kill the thread as we leave
		'''
		self._bus.close()
		self.stop()


	def model (self):
		'''
		Returns a string representing the model
		'''
		return self._model


	@abc.abstractmethod
	def _read_voltage (self):
		'''
		Reads the battery voltage in mV from the device registers
		'''
		pass


	def voltage (self):
		'''
		Returns the voltage.
		This is an instantaneous measurement, so it won't be precise
		'''
		v = self._read_voltage ()
		return v / 1000.0


	def voltage_avg (self):
		'''
		Returns the average voltage in the last UPDATE_INTERVAL * HISTORY_LEN seconds
		'''
		return self._avg_voltage


	@abc.abstractmethod
	def _read_output_current (self):
		'''
		Reads the current drawn in mA from the device registers
		'''
		pass


	def output_current (self):
		'''
		Returns the current drawn in Amps.
		This is an instantaneous measurement, so it won't be precise
		'''
		i = self._read_output_current ()
		return i / 1000.0


	def output_current_avg (self):
		'''
		Returns the average voltage in the last UPDATE_INTERVAL * HISTORY_LEN seconds
		'''
		return self._avg_output_current


	@abc.abstractmethod
	def _read_temperature(self):
		'''
		Reads the temperature from the device registers
		'''
		pass


	# this should be changed based on the board
	def temperature (self):
		'''
		Returns the chip temperature in celsius.
		This is an instantaneous measurement, so it won't be precise
		'''
		t = self._read_temperature()
		return t


	# staticmethod is an inbuilt decorator that defines 
	# a method as static inside a class
	@staticmethod 
	def _convert_battery_voltage_to_level (battery_voltage, battery_curve):
		'''
		Returns the battery percentage given the battery voltage curve
		'''
		for i in range(len(battery_curve)):
			voltage_low = battery_curve[i][0]
			level_low 	= battery_curve[i][1]
			if battery_voltage > voltage_low:
				if i == 0:
					return level_low
				else:
					voltage_high = battery_curve[i - 1][0]
					level_high = battery_curve[i - 1][1]
					percent = (battery_voltage - voltage_low) / (voltage_high - voltage_low)
					return level_low + percent * (level_high - level_low)
		return 0.0


	def percent(self):
		'''
		Returns the battery percentage for the PiSugar3/PiSugar2 Pro (?)
		This is an instantaneous measurement, so it won't be precise
		'''
		battery_level = 100.0
		v = self.voltage()
		if v > 0.0:
			battery_level = self._convert_battery_voltage_to_level(v, self._battery_curve)
		return battery_level


	def avg_percent (self):
		'''
		'''
		return self._avg_percent

	
	@abc.abstractmethod
	def is_power_plugged (self):
		'''
		Self explanatory
		'''
		pass


	@abc.abstractmethod
	def is_charging_allowed (self):
		'''
		Self explanatory
		'''
		pass


	@abc.abstractmethod
	def toggle_allow_charging (self, enable: bool):
		'''
		Enable/Disable charging
		'''
		pass


	@abc.abstractmethod
	def is_charging (self):
		'''
		Duh
		'''
		pass


	@abc.abstractmethod
	def toggle_power_restore (self, auto_restore: bool):
		'''
		Power restore: turn on when the external power supply is restored
		'''
		pass


	@abc.abstractmethod
	def toggle_soft_poweroff (self, enable: bool):
		'''
		Power restore: turn on when the external power supply is restored
		'''
		pass
