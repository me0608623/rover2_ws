# #!/usr/bin/env python3
# import sys
# import rclpy
# from campusrover_msgs.msg import WorkingFloor
# from campusrover_msgs.srv import RoutingPath
# from nav_msgs.msg import Path
# from campusrover_msgs.srv import PlannerFunction
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Bool
# from campusrover_msgs.srv import demoPath
# # from campusspot_msgs.srv import mpcPlanner

# class Demo:
#     def __init__(self):
#         self.map_pub_ = rospy.Publisher('working_floor', WorkingFloor, queue_size=10)
#         self.routing_client_ = rospy.ServiceProxy('generation_path', RoutingPath)
#         # self.path_pub_ = rospy.Publisher('global_path',Path, queue_size=10)
#         # self.path_client_ = rospy.ServiceProxy('planner_function', PlannerFunction)
#         # self.mpc_client_ = rospy.ServiceProxy('mpc_server', mpcPlanner)
#         self.demo_srv_ = rospy.Service('navgation_srv',demoPath, self.navgation_callback)

#         self.location_info = WorkingFloor()
#         self.path_data = RoutingPath()
#         self.speed = Twist()
#         self.T = Bool()
#         self.F = Bool()
#         self.T.data = True
#         self.F.data = False
#         self.speed.linear.x = rospy.get_param("max_linear_speed",0.8)
#         self.speed.linear.y = 0
#         self.speed.linear.z = 0
#         self.speed.angular.x = 0
#         self.speed.angular.y = 0 
#         self.speed.angular.z = rospy.get_param("max_angular_speed",0.25)
#         rospy.sleep(1)
#         self.getParameters()
#         self.loadMap()
        

#     def getParameters(self):
#         self.location_info.building = rospy.get_param("start_building",'itc')
#         self.location_info.floor = rospy.get_param('start_floor', '3')

#     def loadMap(self):
#         self.map_pub_.publish(self.location_info)
    
#     def loadPath(self,start_p, end_p):
#         self.path_data = (self.routing_client_(start_p,[end_p])).routing[0]
#         # self.path_pub_.publish(self.path_data)

#     def navgation_callback(self,req):
#         print(req.start_point,'~~~~~~~~~~~',req.end_point)
#         self.loadPath(req.start_point, req.end_point) ######### define in srv
#         # self.path_client_(self.T,self.F,self.F,1,self.speed)
#         self.mpc_client_(self.path_data, True)
    
        
        
        

# if __name__ == "__main__":
#     rospy.init_node('demo_campusrover')
#     d = Demo()
#     rospy.spin()
    
     

#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from campusrover_msgs.msg import WorkingFloor
from campusrover_msgs.srv import RoutingPath, PlannerFunction, DemoPath
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

class Demo(Node):
    def __init__(self):
        super().__init__('demo_campusrover')
        self.map_pub_ = self.create_publisher(WorkingFloor, 'working_floor', 10)
        self.routing_client_ = self.create_client(RoutingPath, 'generation_path')
        self.path_pub_ = self.create_publisher(Path, 'global_path', 10)
        self.path_client_ = self.create_client(PlannerFunction, 'planner_function_dwa')
        # self.mpc_client_ = self.create_client(mpcPlanner, 'mpc_server')
        self.demo_srv_ = self.create_service(DemoPath, 'navgation_srv', self.navgation_callback)

        self.location_info = WorkingFloor()
        self.path_data = None
        self.speed = Twist()
        self.T = Bool()
        self.F = Bool()
        self.T.data = True
        self.F.data = False

        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 0.25)
        self.declare_parameter("start_building", 'itc')
        self.declare_parameter("start_floor", '3')

        self.speed.linear.x = self.get_parameter("max_linear_speed").value
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = self.get_parameter("max_angular_speed").value

        self.getParameters()
        self.loadMap()

    def getParameters(self):
        self.location_info.building = self.get_parameter("start_building").value
        self.location_info.floor = self.get_parameter("start_floor").value

    def loadMap(self):
        self.map_pub_.publish(self.location_info)
    
    # def loadPath(self, start_p, end_p):
    #     # 等待 service 可用
    #     self.get_logger().info("1")
    #     if not self.routing_client_.wait_for_service(timeout_sec=2.0):
    #         self.get_logger().error('generation_path service not available!')
    #         return
    #     req = RoutingPath.Request()
    #     req.origin = start_p
    #     req.destination = [end_p]
    #     # future = self.routing_client_.call_async(req)
    #     # self.get_logger().info("2")
    #     # rclpy.spin_until_future_complete(self, future)
    #     # self.get_logger().info("3")
        
    #     # 發送 async request
    #     future = self.routing_client_.call_async(req)
    #     self.get_logger().info("Service request sent")

    #     # 同一 function 內非阻塞等待 response
    #     while rclpy.ok():
    #         rclpy.spin_once(self, timeout_sec=0.1)  # 處理 callback
    #         if future.done():  # 若收到 response
    #             try:
    #                 result = future.result()
    #                 if not result.routing:
    #                     self.get_logger().error("Empty path received")
    #                     return

    #                 # 將 path 直接 publish
    #                 path_msg = result.routing[0]  # 已經是 nav_msgs/Path
    #                 self.path_pub_.publish(path_msg)
    #                 self.get_logger().info("Published global path to RViz2")
    #             except Exception as e:
    #                 self.get_logger().error(f"Service call failed: {e}")
    #             break  # 跳出 while
    def loadPath(self, start_p, end_p):
        self.get_logger().info("1")
        if not self.routing_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('generation_path service not available!')
            return

        req = RoutingPath.Request()
        req.origin = start_p
        req.destination = [end_p]

        future = self.routing_client_.call_async(req)
        future.add_done_callback(self._path_response_callback)

        self.get_logger().info("Service request sent")
        
    def _path_response_callback(self, future):
        try:
            result = future.result()
            if not result.routing:
                self.get_logger().error("Empty path received")
                return

            # nav_msgs/Path
            self.path_data = result.routing[0]
            self.path_pub_.publish(self.path_data)
            self.get_logger().info("Published global path to RViz2")
            self.setPlanner()  # 路徑取得後呼叫 setPlanner
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def setPlanner(self):
        if self.path_data is None:
            self.get_logger().error("No path data available for planning")
            return

        if not self.path_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('planner_function service not available!')
            return

        req = PlannerFunction.Request()
        req.action = self.T
        req.direction_inverse = self.F
        req.obstacle_avoidance = self.T
        req.mode = PlannerFunction.Request.MODE_GLOBAL_PATH
        req.speed_parameter = self.speed
        future = self.path_client_.call_async(req)
        future.add_done_callback(self._planner_response_callback)
    
    def _planner_response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"PlannerFunction response: {result}")
        except Exception as e:
            self.get_logger().error(f"PlannerFunction call failed: {e}")
        
    def navgation_callback(self, req, resp):
        self.get_logger().info(f"{req.start_point} ~~~~~~~~~~~ {req.end_point}")
        self.loadPath(req.start_point, req.end_point)
        
        # 如果有 path_client_ 或 mpc_client_，請用 ROS 2 client 寫法呼叫
        # 例如: self.path_client_.call_async(...)
        # 例如: self.mpc_client_.call_async(...)
        # resp 可根據 demoPath.srv 定義填入
        return resp

def main(args=None):
    rclpy.init(args=args)
    d = Demo()
    rclpy.spin(d)
    rclpy.shutdown()

if __name__ == "__main__":
    main()