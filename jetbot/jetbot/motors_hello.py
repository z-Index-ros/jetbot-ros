from board import SCL, SDA
import time
from adafruit_motorkit import MotorKit

"""
# use the following line to specify the I2C bus (0 (default) or 1)
import busio
i2c = busio.I2C(SCL, SDA)

# the pca9685 microcontroller is at address 0x60
kit = MotorKit(i2c=i2c, address=0x60)
"""

kit = MotorKit() # use the default bus and address

# throttle the motor 1 wait a while then stop
kit.motor1.throttle = 1.0
time.sleep(0.5)
kit.motor1.throttle = 0
