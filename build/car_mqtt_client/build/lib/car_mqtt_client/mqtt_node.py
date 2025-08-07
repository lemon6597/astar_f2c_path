import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import paho.mqtt.client as mqtt
import base64

class ImageMQTTPublisher(Node):
    def __init__(self):
        super().__init__('image_mqtt_publisher')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/zed2i/zed_node/left/image_rect_color/compressed',
            self.listener_callback,
            10)
        
        self.client = mqtt.Client()
        self.client.connect('localhost', 1883, 60)  # 確保port是開的
        self.client.loop_start()

    def listener_callback(self, msg):
        b64 = base64.b64encode(msg.data).decode('utf-8')
        self.client.publish("zed/image", b64)

def main(args=None):
    rclpy.init(args=args)
    node = ImageMQTTPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
