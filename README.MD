# Jetbot running with ROS2

The [JetBot](https://jetbot.org/) has been developped based on a Jetson Nano by the NVidia guys. The project has been build on the Jetpack 4.5 and proposes the JetBot SD image, all-in-one.

The same team proposed then a version based on ROS2 [JetBot ROS](https://github.com/dusty-nv/jetbot_ros)

This project is running on ROS2 Foxy in a Docker container.

___Yes, but now, why this project then ?___

I wanted to re-create step by step the JetBot without depending on any JetPacks, SD image nor Docker container. 
I better learn when doing by myself than applying out-of-the-box receips.
So if you're like me, welcome to this project.


## Prerequisites

Firstly we want to run the Jetson on a quite recent release of Ubuntu and ROS2.

>The target : Ubuntu 20.04 and ROS Galaxy 

### Ubuntu 20.04

Installing Ubuntu 20.04 on a Jetson Nano is quite straightforward is you follow the excellent article [Install Ubuntu 20.04 on Jetson Nano](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html) from Q-engineering team.

> If you want to shorten the time spent to install, compile ... the baseline is also proposed as a SD image [Jetson-Nano-Ubuntu-20-image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)
>
> Yes, I told that I prefer to do it by myself, ... and that's what I did, but prepare to be patient, a wise man is worth two!

### ROS2



``` bash
#pip3 install adafruit-circuitpython-lis3dh
#pip3 install adafruit-circuitpython-motorkit

# AdaFruit Blinka, the lib to use AdaFruit CircuitPython on a Jetson nano
pip3 install adafruit-blinka

# feather
sudo pip3 install adafruit-circuitpython-servokit

# PiOLED
sudo pip3 install adafruit-circuitpython-ssd1306
```

https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library

## Adafruit DC & Stepper Motor HAT
https://fr.pinout.xyz/pinout/adafruit_dc_stepper_motor_hat#

``` bash
    git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
    cd Adafruit-Motor-HAT-Python-Library
    sudo apt-get install python-dev
    sudo python setup.py install
```

## Adafruit Python GPIO Library
https://github.com/adafruit/Adafruit_Python_GPIO

## AdaFruit PiOLED
https://learn.adafruit.com/adafruit-pioled-128x32-mini-oled-for-raspberry-pi/usage
