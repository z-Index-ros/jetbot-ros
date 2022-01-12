# Turn on the motors

The motors and the microcontroller from the bill of materials of the JetBos.org are [Adafruit DC Gearbox Motor - "TT Motor" - 200RPM - 3 to 6VDC](https://www.adafruit.com/product/3777) and an [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927)


> Adafruit references used [Adafruit Stepper + DC Motor FeatherWing](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/circuitpython) and [CircuitPython Usage](https://github.com/adafruit/Adafruit_CircuitPython_PCA9685)

As the Adafruit Blinka (lib for Jetson) was installed in the previous [PiOLED](PiOLED.md) page, we only need to install the motor libraries.

## Install the Python libraries

``` bash
# CircuitPython Installation of MotorKit and Necessary Libraries
sudo pip3 install adafruit-circuitpython-motorkit
sudo pip3 install adafruit-circuitpython-motor

# if not installed, install the microcontroller lib too (shoud have been installed as dependency of the motorkit)
sudo pip3 install adafruit-circuitpython-pca9685
```

> NB: werify the use a version of the Motor lib >= 3.3.3 
>
> See [3.3.3 - Jetson Nano Fix](https://github.com/adafruit/Adafruit_CircuitPython_Motor/releases)
>
> Sometimes a good kick is helpful : ```sudo pip3 install 'adafruit-circuitpython-motor==3.3.4' --force-reinstall```

## Check the I2C motors controller's address

As explained [here](https://learn.adafruit.com/circuitpython-libraries-on-linux-and-the-nvidia-jetson-nano?view=all#enable-uart-i2c-and-spi-3039597-21) you can check the I2C address of the controller

``` bash
i2cdetect -r -y 1

     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: 60 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- -- 

``` 

* 0x3c is the PiOLED
* 0x60 (96) is the PCA9685 microcontroller

# Hello World!

And now we can check that the motor is running. Run the [motors_hello.py](../jetbot/jetbot/motors_hello.py), the motor 1 (left one) should run during half a second.

``` bash
python3 motors_hello.py
``` 

> Additional info on the MotorKit API: [Adafruit MotorKit Library ](https://circuitpython.readthedocs.io/projects/motorkit/en/latest/api.html)


# Welcome back Robot!

Now it's time to re-build the JetBot's Robot class based on our new setup, see [Robot.py](../jetbot/jetbot/Robots.py) and the [test](../jetbot/jetbot/RobotTest.py).

``` bash
python3 RobotTest.py
```  
> We can know run the JetBot forward and backward, turn left and right, and of course ... stop.

# Let's create a ROS2 node to pilot the motors

## Motor node

Using the [Robot class](../jetbot/jetbot/Robots.py) created and tested at the previous step, let's make a node that's going to listen to a topic publishing commands like 'forward', 'stop', ...

We'll later create / reuse a teleop node to pilot the motors using the keyboard.

Let's assume we're using the following convention to pilot:

| action | key |
| --- | ---| 
| forward | r |
| turn left | d |
| turn right | g |
| backward | v |
| stop | f (or any other key) |

Look into the [MotorsSubscriber class](../jetbot/jetbot/motors_node.py) implementation. There's no rocket science here, it's a ROS2 subscriber node to the __key_vel__ topic with a callback that uses the Robot class instance according to the key pressed.

## Teleop node

And now we need a [teleop node](../jetbot/jetbot/motors_teleop.py) to pilot.

The code is quite straightforward, we use the [readkeys](https://pypi.org/project/readkeys/) library to fetch keys pressed. 

``` bash
pip install readkeys
```  


It's a ROS2 publisher node with a timer that triggers regularly to fetch any key pressed and that publishes to the __key_vel__ topic the key that has been pressed

## Build, install and run

Open and terminal window and source ROS2

``` bash
# build form the root of the ROS2 workspace
colcon build

# install
.  install/setup.bash 

# and run the motors node
ros2 run jetbot motors_node
``` 

Open another terminal window and source ROS2

``` bash
# and run the telop node
ros2 run jetbot motors_teleop
``` 

Have fun :-)





