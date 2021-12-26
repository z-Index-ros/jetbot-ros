# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2017 James DeVito for Adafruit Industries
# SPDX-License-Identifier: MIT

# This example is for use on (Linux) computers that are using CPython with
# Adafruit Blinka to support CircuitPython libraries. CircuitPython does
# not support PIL/pillow (python imaging library)!

import time
import subprocess

import rclpy
from rclpy.node import Node

from board import SCL, SDA
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

class PiOLedStats(Node):

    def __init__(self):
        super().__init__(node_name = 'pioled_stats')
        self.create_timer(1, self.timer_callback)

        # Create the I2C interface.
        i2c = busio.I2C(SCL, SDA)

        # Create the SSD1306 OLED class.
        # The first two parameters are the pixel width and pixel height.  Change these
        # to the right size for your display!
        self.disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

        # Clear display.
        self.disp.fill(0)
        self.disp.show()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height


    def timer_callback(self):

        # Load default font.
        self.font = ImageFont.load_default()


        image = Image.new("1", (self.width, self.height))

        # Get drawing object to draw on image.
        draw = ImageDraw.Draw(image)

        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        top = padding
        bottom = self.height - padding
        # Move left to right keeping track of the current x position for drawing shapes.
        x = 0

        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        # Shell scripts for system monitoring from here:
        # https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "hostname -I | cut -d' ' -f1"
        IP = subprocess.check_output(cmd, shell=True).decode("utf-8")
        cmd = 'cut -f 1 -d " " /proc/loadavg'
        CPU = subprocess.check_output(cmd, shell=True).decode("utf-8")
        cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%s MB  %.2f%%\", $3,$2,$3*100/$2 }'"
        MemUsage = subprocess.check_output(cmd, shell=True).decode("utf-8")
        cmd = 'df -h | awk \'$NF=="/"{printf "Disk: %d/%d GB  %s", $3,$2,$5}\''
        Disk = subprocess.check_output(cmd, shell=True).decode("utf-8")

        # Write four lines of text.
        self.get_logger().info("IP: " + IP + " CPU: " + CPU + MemUsage + " " + Disk)

        draw.text((x, top + 0), "IP: " + IP, font=self.font, fill=255)
        draw.text((x, top + 8), "CPU load: " + CPU, font=self.font, fill=255)
        draw.text((x, top + 16), MemUsage, font=self.font, fill=255)
        draw.text((x, top + 25), Disk, font=self.font, fill=255)

        # Display image.
        self.disp.image(image)
        self.disp.show()

    def __del__(self):
        self.get_logger().info("destroy method called, clearing display")
        # Clear display.
        self.disp.fill(0)
        self.disp.show()




def main(args=None):
    rclpy.init(args=args)

    node = PiOLedStats()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
