# ToF sensors calibration

Before going on with the installation of the code it is necessary to configure the sensors. Being identical they will have the same address on the I2C bus. It is necessary to connect them to the RPi, start the script and select the new address to be associate them, which will be stored in their EPROM.
To do so, i'll use [these API](https://github.com/pimoroni/vl53l1x-python). Dependencies: `smbus2`, `vl53l1x`. At this point `smbus2` should already be installed.