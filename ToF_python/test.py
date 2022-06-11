#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
import VL53L0X
import RPi.GPIO as GPIO

# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 20
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 16
# GPIO for Sensor 3 shutdown pin
sensor3_shutdown = 25

GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)
GPIO.setup(sensor3_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)
GPIO.output(sensor3_shutdown, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
GPIO.output(sensor1_shutdown, GPIO.HIGH) # turn on the first one
tof1 = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
tof1.change_address(0x2B)
# tof = VL53L0X.VL53L0X(i2c_address=0x2B)
tof1.open()
print("First address set to 0X2B")

time.sleep(0.50)

GPIO.output(sensor2_shutdown, GPIO.HIGH) # turn on the second one
tof2 = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
tof2.change_address(0x2C)
# tof2 = VL53L0X.VL53L0X(i2c_address=0x2D)
tof2.open()
print("Second address set to 0X2C")

time.sleep(0.50)

GPIO.output(sensor3_shutdown, GPIO.HIGH) # turn on the third one
tof3 = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
tof3.change_address(0x2D)
# tof3 = VL53L0X.VL53L0X(i2c_address=0x2D)
tof3.open()
print("Third address set to 0X2D")

time.sleep(0.50)

# Set shutdown pin high for the first VL53L0X then
# call to start ranging
#GPIO.output(sensor1_shutdown, GPIO.HIGH)
#time.sleep(0.50)
tof1.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
print("First sensor started ranging")

# Set shutdown pin high for the second VL53L0X then
# call to start ranging
#GPIO.output(sensor2_shutdown, GPIO.HIGH)
#time.sleep(0.50)
tof2.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
print("Second sensor started ranging")

tof3.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
print("Third sensor started ranging")

timing = tof1.get_timing() # same for all three of them
if timing < 20000:
    timing = 20000
print("Timing %d ms" % (timing/1000))

# ranging

error = -40

import time

interval = 15 # seconds
t_end = time.time() + interval
while time.time() < t_end:

    distance_1 = tof1.get_distance() + error
    if distance_1 < 0:
        distance_1 = 0

    distance_2 = tof2.get_distance() + error
    if distance_2 < 0:
        distance_2 = 0

    distance_3 = tof3.get_distance() + error
    if distance_3 < 0:
        distance_3 = 0

    # '{:4d}'.format(...) -> format digit over 4 places using ' ' as padding char
    # '{:6.2f}'.format(...) -> format float over 6 places using ' ' as padding char
    #                           rounding to two decimal places
    print('Sensor[1]:\t{:3d}mm\t{:6.2f}cm\t'.format(distance_1, distance_1/10.0), end='')
    print('Sensor[2]:\t{:3d}mm\t{:6.2f}cm\t'.format(distance_2, distance_2/10.0), end='')
    print('Sensor[3]:\t{:3d}mm\t{:6.2f}cm'.format(distance_3, distance_3/10.0))


tof1.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)
tof2.stop_ranging()
GPIO.output(sensor2_shutdown, GPIO.LOW)
tof3.stop_ranging()
GPIO.output(sensor3_shutdown, GPIO.LOW)


tof1.close()
tof2.close()
tof3.close()

