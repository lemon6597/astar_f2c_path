import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
 
class Subscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        self.subscriber = self.create_subscription(String,
                                                  "talker_topic",
                                                  self.sub_callback,
                                                   10)
        
    def sub_callback(self,msg):
        self.get_logger().info("I heard: %s" %msg.data)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Subscriber("listener")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()