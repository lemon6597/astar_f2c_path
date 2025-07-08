import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')


        self.Astar_path = None
        self.Field2Cover_path = None

        self.current_path = None

        self.Astar_Subscription = self.create_subscription(Path,
                                                           "/Astar_list",
                                                           self.Astar_callback,
                                                           10)
        self.Field2Cover_Subscription = self.create_subscription(Path,
                                                           "/Field2Cover_list",
                                                           self.Field2Cover_callback,
                                                           10)
        
        # self.get_logger().info('Waiting for action server...')
        # self._client.wait_for_server()
        # self.get_logger().info('Action server available!')
        
    def Astar_callback(self,msg):
        self.Astar_path = msg.poses

        if not self.Astar_path:
            self.get_logger().warn('Astar empty path')
            return
        if self.current_path is None:
            self.send_path(self.Astar_path, path_name="Astar")
    def Field2Cover_callback(self,msg):
        self.Field2Cover_path = msg.poses
        self.get_logger().info("Received Field2Cover path.")

    def send_path(self,poses,path_name=""):
        if not poses:
            self.get_logger().warn("send empty path")
            self.current_path = None
            return

        self.current_path = path_name

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info(f'Sending goal with {len(poses)} waypoints...')
        self._send_goal(goal_msg)
    
    def _send_goal(self,goal_msg):
        self._send_goal_future = self._client.send_goal_async(goal_msg)      #catch async
        self._send_goal_future.add_done_callback(self.goal_response_callback)#when catch do goal_response_callback

    def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected by server.')
                self.current_path = None
                return
            
            self.get_logger().info('Goal accepted, waiting for result...')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'FollowWaypoints finished with result: {result}')

        if self.current_path == "Astar":
            if self.Field2Cover_path:
                self.send_path(self.Field2Cover_path, path_name="Field2Cover")
                self.Field2Cover_path = None
            else:
                self.get_logger().info("Field2Cover ERROR")
                self.current_path = None
        elif self.current_path =="Field2Cover":
            self.get_logger().info("Field2Cover goal")
            self.current_path = None
            self.Field2Cover_path = None
            self.Astar_path = None
            self.get_logger().info("Reset")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()