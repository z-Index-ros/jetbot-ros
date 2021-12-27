![](https://jetbot.org/master/images/jetbot_800x630.png)

# Jetbot running with ROS2

The [JetBot](https://jetbot.org/) has been developped based on a Jetson Nano by the NVidia guys. The project has been build on the Jetpack 4.5 and proposes the JetBot SD image, all-in-one.

The same team proposed then a version based on ROS2 [JetBot ROS](https://github.com/dusty-nv/jetbot_ros)

This project is running on ROS2 Foxy in a Docker container.

___Yes, but now, why this project then ?___

I wanted to re-create step by step the JetBot without depending on any JetPacks, SD image nor Docker container. 
I better learn when doing by myself than applying out-of-the-box receips.
So if you're like me, welcome to this project.


## Prerequisites

### Hardware prerequisite

Your Jetbot is mounted and operational: [JetBot Hardware Setup](https://jetbot.org/master/hardware_setup.html). 

> That should be the case otherwise you'd not be ready these lines.

### Software prerequisites
Firstly we want to run the Jetson on a quite recent release of Ubuntu and ROS2.

>The target : Ubuntu 20.04 and ROS2 Galaxy 

#### Installing Ubuntu 20.04

Installing Ubuntu 20.04 on a Jetson Nano is quite straightforward is you follow the excellent article [Install Ubuntu 20.04 on Jetson Nano](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html) from Q-engineering team.

> If you want to shorten the time spent to install, compile ... the baseline is also proposed as a SD image [Jetson-Nano-Ubuntu-20-image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)
>
> Yes, I told that I prefer to do it by myself, ... and that's what I did, but prepare to be patient, a wise man is worth two!

#### Installing ROS2

Follow the [Guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html).

Perform some quick verifications to be sure that the ROS2 is up-n-running, like running the tutorials.

## Building the JetBot on ROS2 Setp-by-step

* [Turn on the PiOLED](./doc/PiOLED.md)
