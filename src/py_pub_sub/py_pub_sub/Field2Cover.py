import fields2cover as f2c
import rclpy
import csv
from geometry_msgs.msg import PoseStamped
import tf_transformations
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from rclpy.node import Node
import math


class Subscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        self.last_leaf = None
        self.box_size = 10.0              # 四邊形邊長
        self.robot_c = f2c.Robot(0.28,0.6,0.28);

        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_yaw = 0.0

        self.create_subscription(PoseStamped,
                                '/robot_pose',
                                self.pose_callback, 
                                10)
        self.point_Subscriber = self.create_subscription(Float64MultiArray,
                                                         "point_data",
                                                         self.point_callback,
                                                         10)
        self.path_pub = self.create_publisher(Path,
                                              "/Field2Cover_list",
                                              10)
    def pose_callback(self, msg):
        self.current_pose_x = msg.pose.position.x
        self.current_pose_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
                                                         
    def point_callback(self,msg):
        data = msg.data
        size = 6
        num = len(data)//size

        for i in range(num):
            Base = i*size
            leaf_x = data[Base]  #frist
            leaf_y = data[Base+1] #second

        if self.last_leaf == (leaf_x, leaf_y):
            return
        
        self.last_leaf = (leaf_x, leaf_y)
        self.count_path(leaf_x, leaf_y)
        

    def count_path(self,leaf_x,leaf_y):
        field = f2c.Cells(
            f2c.Cell(
                f2c.LinearRing(
                    f2c.VectorPoint([
                        f2c.Point(leaf_x, leaf_y),
                        f2c.Point(leaf_x + self.box_size, leaf_y),
                        f2c.Point(leaf_x + self.box_size, leaf_y + self.box_size),
                        f2c.Point(leaf_x, leaf_y + self.box_size),
                        f2c.Point(leaf_x, leaf_y)  # 關閉回到起點
                ])
            )
        )
    )
        robot_x = self.current_pose_x
        robot_y = self.current_pose_y
        yaw = self.current_yaw  # 弧度制
        start_pose = f2c.Point(robot_x, robot_y)

        const_hl = f2c.HG_Const_gen();
        no_hl_c = const_hl.generateHeadlands(field, 1* self.robot_c.getWidth());
       
        bf = f2c.SG_BruteForce();

        swaths = bf.generateSwaths(math.pi, self.robot_c.getCovWidth(), no_hl_c.getGeometry(0))

        boustrophedon_sorter = f2c.RP_Boustrophedon();
        swaths = boustrophedon_sorter.genSortedSwaths(swaths);  ###########


        r = 0
        self.robot_c.setMinTurningRadius(r)  # m
        self.robot_c.setMaxDiffCurv(0);  # )1/m^2


        path_planner = f2c.PP_PathPlanning()
        dubins = f2c.PP_DubinsCurves()
        path = path_planner.planPath(self.robot_c, swaths, dubins,start_pose)

        csv_path = "/tmp/path.csv"
        path.saveToFile(csv_path);
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = []
        with open(csv_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                except ValueError:
                    continue
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0  # 預設不旋轉
                path_msg.poses.append(pose)
            if len(path_msg.poses) == 0:
                self.get_logger().warn("Generated path is empty!")
                return


        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published Field2Cover path with {len(path_msg.poses)} poses.")


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber('Field2Cover')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
