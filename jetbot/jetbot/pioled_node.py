import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from board import SCL, SDA
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont


class PiOLedSubscriber(Node):

    def __init__(self):
        super().__init__(node_name = 'pioled_subscriber')
        self.subscription = self.create_subscription(String, 'hello_topic', self.listener_callback, 10)
        self.subscription
        

        # Create the I2C interface.
        i2c = busio.I2C(SCL, SDA)

        # Create the SSD1306 OLED class.
        # The first two parameters are the pixel width and pixel height.  Change these
        # to the right size for your display!
        self.disp = adafruit_ssd1306.SSD130leo6_I2C(128, 32, i2c)

        # Clear display.
        self.disp.fill(0)
        self.disp.show()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height

        # Load default font.
        self.font = ImageFont.load_default()

    def get_wrapped_text(self, text: str):#, font: ImageFont.ImageFont, line_length: int):
        lines = ['']
        for word in text.split():
            line = f'{lines[-1]} {word}'.strip()
            if self.font.getsize(line)[0] <= self.width:#line_length:
                lines[-1] = line
            else:
                lines.append(word)
        return '\n'.join(lines)

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        text = msg.data
        
        image = Image.new("1", (self.width, self.height))

        # Get drawing object to draw on image.
        draw = ImageDraw.Draw(image)

        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)


        # draw the text
        draw.multiline_text((0,0), self.get_wrapped_text(text), font=self.font, fill=255)

        # Display image.
        self.disp.image(image)
        self.disp.show()

        self.get_logger().info("pioled text updated")

    def __del__(self):
        self.get_logger().info("destroy method called, clearing display")
        # Clear display.
        self.disp.fill(0)
        self.disp.show()




def main(args=None):
    rclpy.init(args=args)

    subscriber = PiOLedSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        subscriber.destroy_node()

if __name__ == '__main__':
    main()


