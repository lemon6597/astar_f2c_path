import fields2cover as f2c
import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

class Subscriber(Node):
    def __init__(self,name):
        super().__init__(self,name)
        self.point_Subscriber = self.create_subscription(Float64MultiArray,
                                                         "pont_data",
                                                         self.point_callback,
                                                         10)
    def point_callback(self,msg):
        data = msg.data
        size = 6
        num = len(data)//size

        for i in range(num):
            Base = i*size
            x = data[Base]  #frist
            y = data[Base+1] #second
        





def main(args=None):
    rclpy.init(args=args)
    node = Subscriber('Field2Cover')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
