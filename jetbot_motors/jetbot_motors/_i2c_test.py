import time
import _Adafruit_PWM_Servo_Driver as _Adafruit_PWM_Servo_Driver
from Adafruit_MotorHAT import Adafruit_MotorHAT

addr = 0x60

i2c = _Adafruit_PWM_Servo_Driver.get_i2c_device(addr, None, 1)
print(i2c)

pwm = _Adafruit_PWM_Servo_Driver.PWM(address=addr, i2c=None, i2c_bus=1)
print (pwm)

mh = Adafruit_MotorHAT(addr=addr, i2c_bus=1)
print(mh)

left = mh.getMotor(1)
print(left)

left.setSpeed(100)
left.run(Adafruit_MotorHAT.FORWARD)
time.sleep(1)
left.run(Adafruit_MotorHAT.RELEASE)
