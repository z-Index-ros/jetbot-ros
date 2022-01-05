from board import SCL, SDA
import busio
import time
import adafruit_blinka
from adafruit_motorkit import MotorKit

from adafruit_blinka.agnostic import detector

print(detector.board.any_jetson_board)

#print (adafruit_blinka.microcontroller)

# https://githubmate.com/repo/adafruit/Adafruit_CircuitPython_Motor/activity?page=2
# sudo pip3 install 'adafruit-circuitpython-motor==3.3.1' --force-reinstall
i2c = busio.I2C(SCL, SDA)

kit = MotorKit()#i2c=i2c, address=0x60)

kit.motor1.throttle = 1.0
time.sleep(0.5)
kit.motor1.throttle = 0

""" 
Adafruit-Blinka                      6.17.0              
adafruit-circuitpython-busdevice     5.1.1               
adafruit-circuitpython-framebuf      1.4.8               
adafruit-circuitpython-lis3dh        5.1.12              
adafruit-circuitpython-motor         3.3.4               
adafruit-circuitpython-motorkit      1.6.3               
adafruit-circuitpython-pca9685       3.3.9               
adafruit-circuitpython-register      1.9.6               
adafruit-circuitpython-servokit      1.3.6               
adafruit-circuitpython-ssd1306       2.12.3              
Adafruit-GPIO                        1.0.4               
Adafruit-MotorHAT                    1.4.0               
Adafruit-PCA9685                     1.0.1               
Adafruit-PlatformDetect              3.18.0              
Adafruit-PureIO                      1.1.9      
"""          
