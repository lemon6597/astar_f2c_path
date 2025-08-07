import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import paho.mqtt.client as mqtt
import base64
import threading
from flask import Flask, send_from_directory


HTML_FOLDER = "/home/humble/ros2_ws/src/car_mqtt_client/car_mqtt_client"  # 請改成你真實路徑
HTML_FILE = "leaves404.html"
HTTP_PORT = 8080

app = Flask(__name__)

@app.route('/')
def serve_html():
    return send_from_directory(HTML_FOLDER, HTML_FILE)

class ImageMQTTPublisher(Node):
    def __init__(self):
        super().__init__('image_mqtt_publisher')

        self.subscription_1 = self.create_subscription(
            CompressedImage,
            '/zed2i/zed_node/left/image_rect_color/compressed',
            self.zed1_callback,
            10)
        
        self.subscription_2 = self.create_subscription(
            CompressedImage,
            '/zed/zed_node/left/image_rect_color/compressed',
            self.zed2_callback,
            10)
        
        self.client = mqtt.Client()
        self.client.connect('localhost', 1883, 60)  # 確保port是開的
        self.client.loop_start()
        self.get_logger().info("Start MQTT")


        flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=HTTP_PORT))
        flask_thread.daemon = True
        flask_thread.start()

        self.get_logger().info(f"HTTP Server started on port {HTTP_PORT}")

    def zed1_callback(self, msg):
        b64_1 = base64.b64encode(msg.data).decode('utf-8')
        self.client.publish("zed2i/image", b64_1)

    def zed2_callback(self, msg):
        b64_2 = base64.b64encode(msg.data).decode('utf-8')
        self.client.publish("zed/image", b64_2)

def main(args=None):
    rclpy.init(args=args)
    node = ImageMQTTPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
