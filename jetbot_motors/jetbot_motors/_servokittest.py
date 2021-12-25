from adafruit_servokit import ServoKit

kit = ServoKit(channels=16, address=0x60)
kit.servo[0].angle=173
kit.servo[0].angle=25
