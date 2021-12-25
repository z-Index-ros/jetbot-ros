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
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.subscription

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
        image = Image.new("1", (self.width, self.height))

        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(image)

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        # Load default font.
        self.font = ImageFont.load_default()

    def get_wrapped_text(text: str, font: ImageFont.ImageFont,
                        line_length: int):
            lines = ['']
            for word in text.split():
                line = f'{lines[-1]} {word}'.strip()
                if font.getsize(line)[0] <= line_length:
                    lines[-1] = line
                else:
                    lines.append(word)
            return '\n'.join(lines)

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)

        # draw the text
        self.draw.multiline_text((0,0), self.get_wrapped_text(msg.data, self.font, self.width), font=self.font, fill=255)

        # Display image.
        self.disp.image(self.image)
        self.disp.show()

        print("pioled text updated")

def main(args=None):
    rclpy.init(args=args)

    subscriber = PiOLedSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


