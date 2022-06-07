import VL53L1X

# Open and start the VL53L1X sensor.
# If you've previously used change-address.py then you
# should use the new i2c address here.
# If you're using a software i2c bus (ie: HyperPixel4) then
# you should `ls /dev/i2c-*` and use the relevant bus number.
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()

# Optionally set an explicit timing budget
# These values are measurement time in microseconds,
# and inter-measurement time in milliseconds.
# If you uncomment the line below to set a budget you
# should use `tof.start_ranging(0)`
# tof.set_timing(66000, 70)

tof.start_ranging(1)  # Start ranging
                      # 0 = Unchanged
                      # 1 = Short Range
                      # 2 = Medium Range
                      # 3 = Long Range

# loop for 10 seconds
import time

interval_s = 10

# time.time() returns the current time in seconds
t_end = time.time() + interval_s

while time.time() < t_end:

    try:

        # Grab the range in mm, this function will block until
        # a reading is returned.
        distance_in_mm = tof.get_distance()
        print("distance\t{:.2f} [mm]\t{:.2f} [cm]".format(distance_in_mm, distance_in_mm / 10))
    
    except:
        pass


tof.stop_ranging()