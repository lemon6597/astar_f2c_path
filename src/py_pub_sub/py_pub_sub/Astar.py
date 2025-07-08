import rclpy
import heapq
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped


 
class Subscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.width = None
        self.height = None
        self.origin_x = None
        self.origin_y = None
        self.obstacles = None
        self.position = None

        self.point_subscriber = self.create_subscription(Float64MultiArray,
                                                  "pont_data",
                                                  self.sub_callback,
                                                   10)
        self.map_subscriber = self.create_subscription(OccupancyGrid,
                                                   "/map",
                                                   self.OccupancyGrid_callback,
                                                   10)
        self.pose_publisher = self.create_publisher(Path,
                                                    "Astar_list",
                                                    10)
    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",            # target frame
                "base_link",      # source frame
                rclpy.time.Time(),  # latest available
                timeout=Duration(seconds=1.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)
        except Exception as e:
            return None    
    def OccupancyGrid_callback(self,msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        data = msg.data

        obstacles = set()
        for y in range(self.height):
            for x in range(self.width):

                index = y * self.width + x
                if data[index] > 50:
                    obstacles.add((x, y))


                    
        self.obstacles = obstacles

    def sub_callback(self,msg):
        if (self.width is None or 
            self.height is None or 
            self.resolution is None or 
            self.origin_x is None or 
            self.origin_y is None or 
            self.obstacles is None):
            self.get_logger().warn("Map has not been received yet, skipping path planning.")
            return
        data = msg.data
        size = 6
        num = len(data)//size

        for i in range(num):
            Base = i*size
   
            x = data[Base+4] #box center
            y = data[Base+5] #deep

            #coordinate transform
            grid_x = int((x-self.origin_x)/self.resolution)
            grid_y = int((y-self.origin_y)/self.resolution)
            grid_goal = (grid_x,grid_y)
            robot_pos = self.get_robot_position()
            if robot_pos is None:
                self.get_logger().warn("Robot position unavailable, skipping path planning.")
                return
            grid_start_x = int((robot_pos[0] - self.origin_x) / self.resolution)  #to gridmap
            grid_start_y = int((robot_pos[1] - self.origin_y) / self.resolution)
            self.position = (grid_start_x, grid_start_y)
            
            path = Astar(self.width,self.height,self.position,grid_goal,self.obstacles)
            if path is None:
                self.get_logger().warn(f"No path found from {self.position} to {grid_goal}")
                return
            pose_list = poseStamped_transfor(path, self.resolution, self.origin_x, self.origin_y)

            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = pose_list

            # 發佈
            self.pose_publisher.publish(path_msg)

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
def Astar(grid_width, grid_height, start, goal, obstacles=set()):
    open_list = [] #good st
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start, [start])) #list,f(sum(g)),g,position,path heapq.heappush-->f最小-->g最小
#(12, 1, (2, 0),[(0, 0)])
#(12, 2, (1, 0),[(0, 0)])
#(12, 3, (0, 1),[(0, 0)])...

    visited = set() #紀錄Now position

    while open_list:
        est_total_cost, cost_so_far, current, path = heapq.heappop(open_list) #heapq.heappop-->f最小取出

        if current == goal:  #arrival
            return path      

        if current in visited:
            continue  #skip loop
        visited.add(current) 

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]: #right left under on
            neighbor = (current[0] + dx, current[1] + dy)

            if not (0 <= neighbor[0] < grid_width and 0 <= neighbor[1] < grid_height):
                continue

            if neighbor in obstacles:
                continue

            heapq.heappush(open_list, (
                cost_so_far + 1 + heuristic(neighbor, goal),
                cost_so_far + 1,
                neighbor,
                path + [neighbor]
            ))
    return None

def poseStamped_transfor(path, resolution, origin_x, origin_y, frame_id='map'):
    pose_list = []
    for (grid_x,grid_y) in path:
        pose = PoseStamped()
        pose.header.frame_id = frame_id           # 設定座標系
        pose.header.stamp = Time().to_msg()       # 現在時間

        pose.pose.position.x = grid_x*resolution + origin_x           # x 座標
        pose.pose.position.y = grid_y*resolution + origin_y           # y 座標
        pose.pose.position.z = 0.0                  # z 通常設 0，平面地圖

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        pose_list.append(pose)
    return pose_list
    

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber("Astar")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()