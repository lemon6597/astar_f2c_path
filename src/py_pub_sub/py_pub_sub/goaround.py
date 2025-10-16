import fields2cover as f2c
import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math


pose = [(1.0,2.0),
        (2.0,3.3),
        (3.0,2.3),
        (-2.0,1.0),
        (1.0,2.0),
        ]
class goaround(Node):
    def __init__(self):
        super().__init__('goaround')

        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.point_sub = self.create_subscription(
            Float64MultiArray,
            "point_data",
            self.point_callback,
            10
        )
        self.resume_sub = self.create_subscription(
            Float64MultiArray,
            "resume",
            self.resume_callback,
            10
        )

        # self.get_logger().info("Waiting for Waypoint action")
        # self._client.wait_for_server()
        # self.get_logger().info("Connect!!")

        self.map_list = []
        for i, (x, y) in enumerate(pose):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = 0.0

            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0

            self.map_list.append(p)

        
        self.forward = True
        self.index = 0
        self.stop_flag = False

        self.move_base()

        # goal_msg = FollowWaypoints.Goal()
        # goal_msg.poses = self.map_list
        # self.get_logger().info(f'Sending{len(self.map_list)}waypoints...')
        # self._client.send_goal_async(goal_msg)
    
    def move_base(self):
        goal_pose = self.map_list[self.index]
        if self.forward and self.index < len(self.map_list) - 1:
            next_x, next_y = (self.map_list[self.index + 1].pose.position.x,
                          self.map_list[self.index + 1].pose.position.y)
            yaw = math.atan2(next_y - goal_pose.pose.position.y, next_x - goal_pose.pose.position.x)
        elif not self.forward and self.index > 0:
            next_x, next_y = (self.map_list[self.index - 1].pose.position.x,
                          self.map_list[self.index - 1].pose.position.y)
            yaw = math.atan2(next_y - goal_pose.pose.position.y, next_x - goal_pose.pose.position.x)
        else:
            yaw = 0.0
        
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)


        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses= [goal_pose]

        self.get_logger().info(f"Sending waypoint {self.index}: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")

        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result()
        self.get_logger().info("Reached waypoint")

        if self.stop_flag:
            self.get_logger().info("Paused due to object detection")
            return  

        # 更新 index
        if self.forward:
            self.index += 1
            if self.index >= len(self.map_list):  # 到達最後一個點，翻轉方向
                self.index = len(self.map_list) - 2
                self.forward = False
        else:
            self.index -= 1
            if self.index < 0:  # 到達第一個點，翻轉方向
                self.index = 1
                self.forward = True

        # 發送下一個 waypoint
        self.move_base()
    
    def point_callback(self,msg):
        self.stop_flag = True
        if self._client._goal_handle is not None:
            self._client._goal_handle.cancel_goal_async()


    def resume_callback(self,msg):
        if self.stop_flag:
            self.stop_flag = False
            self.get_logger().info("Resuming goaround...")
            self.move_base()


def main(args=None):
    rclpy.init(args=args)
    node = goaround()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()