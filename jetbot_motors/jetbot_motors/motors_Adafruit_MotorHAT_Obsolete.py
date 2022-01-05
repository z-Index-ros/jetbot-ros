#import rclpy
import time
import board
from board import SCL, SDA
import busio
from adafruit_motorkit import MotorKit

from Adafruit_MotorHAT import Adafruit_MotorHAT

driver = Adafruit_MotorHAT(i2c_bus=1, addr=0x60)

MOTOR_LEFT = driver.getMotor(1)
MOTOR_LEFT.setSpeed(100)
MOTOR_LEFT.run(Adafruit_MotorHAT.FORWARD)
time.sleep(2)
MOTOR_LEFT.setSpeed(0)
MOTOR_LEFT.run(Adafruit_MotorHAT.RELEASE)


""" from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c, address=0x60)
pca.frequency = 100
 """

""" import time
from adafruit_motorkit import MotorKit

kit = MotorKit()

kit.motor1.throttle = 1.0
time.sleep(0.5)
kit.motor1.throttle = 0 """

