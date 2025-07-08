
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class Publisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.publisher = self.create_publisher(String,"talker_topic",10)
        time_stand = 5
        self.timer = self.create_timer(time_stand,self.time_callback)
        self.get_logger().info("hello,I'm %s!" %name)
        self.i = 0

    def time_callback(self):
        msg = String()
        msg.data = 'NO.%d,hello!' %(self.i)
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: %s"%msg.data) 
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = Publisher("talker")
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
