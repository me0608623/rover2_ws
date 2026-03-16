# "mpc_finish : 'true'" 或發布 "stop_navigation : 'true'"才可發新路線
# use "ros2 service call /navgation_srv campusrover_msgs/srv/DemoPath "start_point: '' end_point: ''"

#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from campusrover_msgs.msg import WorkingFloor
from campusrover_msgs.srv import RoutingPath, PlannerFunction, DemoPath, MapClosestLocation
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class Demo(Node):
    def __init__(self):
        super().__init__('demo_campusrover')
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("loc_update_period", 0.5)  # seconds
        
        self.target_frame_ = self.get_parameter("target_frame").value
        self.base_frame_ = self.get_parameter("base_frame").value
        
        self.mpc_finish_sub_ = self.create_subscription(Bool, 'mpc_finish', self._mpc_finish_cb, 10)
        self.stop_nav_sub_ = self.create_subscription(Bool, 'stop_navigation', self._stop_nav_cb, 10)
        
        self.map_pub_ = self.create_publisher(WorkingFloor, 'working_floor', 10)
        self.path_pub_ = self.create_publisher(Path, 'global_path', 10)

        self.routing_client_ = self.create_client(RoutingPath, 'generation_path')
        self.path_client_ = self.create_client(PlannerFunction, 'planner_function_dwa')
        self.mpc_client_ = self.create_client(SetBool, 'enable_bubble_mpc')
        self.closest_client_ = self.create_client(MapClosestLocation, 'get_closest_location') # 目前用不了

        self.demo_srv_ = self.create_service(DemoPath, 'navgation_srv', self.navgation_callback)

        # 狀態：目前推定的起點
        self.current_origin_ = ""  
        self._closest_future = None   # 避免 timer 一直疊 call
        period = float(self.get_parameter("loc_update_period").value)
        self.create_timer(period, self._update_origin_timer)
        
        self.allow_new_route_ = True     # True 才允許 navgation_srv 接新目標
        self.nav_active_ = False
        self.goal_seq_ = 0               # 避免舊 response 覆蓋新目標
        self.latest_seq_ = 0
        
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

    def _mpc_finish_cb(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("mpc_finish == True -> unlock new navigation, disable MPC")
        self._unlock_and_disable_mpc()

    def _stop_nav_cb(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("stop_navigation == True -> unlock new navigation, disable MPC")
        self._unlock_and_disable_mpc()

    def _unlock_and_disable_mpc(self):
        # 允許下一次 navgation_srv 接新路線
        self.allow_new_route_ = True
        self.nav_active_ = False

        # 刷新/關閉 MPC（你要求的 enable_mpc 狀態刷新）
        self.call_enable_mpc(False)

    def _update_origin_timer(self):
        # 避免上一次 service 還沒回來就又 call
        if self._closest_future is not None and not self._closest_future.done():
            return

        # TF: world -> base_footprint
        try:
            tf = self.tf_buffer_.lookup_transform(
                self.target_frame_,
                self.base_frame_,
                Time(),  # latest
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # closest service ready?
        # if not self.closest_client_.service_is_ready():
        #     self.get_logger().warn("closest_client_ not ready")
        #     return

        # call closest service
        req = MapClosestLocation.Request()
        req.building = self.location_info.building
        req.floor = self.location_info.floor

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame_

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(tf.transform.translation.z)

        # orientation 直接用 TF 的 quaternion
        pose.pose.orientation = tf.transform.rotation

        req.robot_pose = pose


        self._closest_future = self.closest_client_.call_async(req)
        self._closest_future.add_done_callback(self._closest_done_cb)


    def _closest_done_cb(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f"get_closest_location failed: {e}")
            return

        # ⚠️ 這裡的 response 欄位要依你的 srv 定義調整
        # 常見是 resp.closest_point 或 resp.name / resp.id
        closest = resp.closest_point

        if isinstance(closest, str) and closest:
            self.current_origin_ = closest
            # 你想看 log 再打開
            # self.get_logger().info(f"current_origin_ updated: {self.current_origin_}")

    def loadPath(self, start_p, end_p, seq: int):
        self.get_logger().info("1")
        if not self.routing_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('generation_path service not available!')
            self.allow_new_route_ = True
            self.nav_active_ = False
            return


        req = RoutingPath.Request()
        req.origin = start_p
        req.destination = [end_p]

        future = self.routing_client_.call_async(req)
        future.add_done_callback(lambda fut, s=seq: self._path_response_callback(fut, s))
        self.get_logger().info(f"generation_path request sent (seq={seq})")

        self.get_logger().info("Service request sent")
        
    def _path_response_callback(self, future, seq: int):
        if seq != self.latest_seq_:
            self.get_logger().warn(f"Ignore outdated routing response seq={seq}, latest={self.latest_seq_}")
            return
        try:
            result = future.result()
            if not result.routing:
                self.get_logger().error("Empty path received")
                self.allow_new_route_ = True
                self.nav_active_ = False
                return

            # nav_msgs/Path
            self.path_data = result.routing[0]
            self.path_pub_.publish(self.path_data)
            self.get_logger().info("Published global path to RViz2")
            self.setPlanner()  # 路徑取得後呼叫 setPlanner
            self.call_enable_mpc(True)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.allow_new_route_ = True
            self.nav_active_ = False

    def call_enable_mpc(self, enable: bool):
        if not self.mpc_client_.service_is_ready():
            self.get_logger().warn('enable_mpc not ready, waiting...')
            if not self.mpc_client_.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('enable_mpc still not available')
                return

        req = SetBool.Request()
        req.data = enable

        future = self.mpc_client_.call_async(req)
        future.add_done_callback(lambda fut, en=enable: self._enable_mpc_done_cb(fut, en))
        
    def _enable_mpc_done_cb(self, future, enable: bool):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'enable_mpc call failed: {e}')
            return

        if resp.success:
            self.get_logger().info(f"enable_mpc({enable}) OK: {resp.message}")
        else:
            self.get_logger().warn(f'enable_mpc rejected: {resp.message}')
    
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

        if not self.allow_new_route_:
            self.get_logger().warn("Navigation is active. Reject new route until mpc_finish/stop_navigation == True.")
            if hasattr(resp, "success"):
                resp.success = False
            if hasattr(resp, "message"):
                resp.message = "busy: wait for mpc_finish or stop_navigation"
            return resp

        # --------- ✅ 終點空字串檢查 ----------
        dest = req.end_point.strip() if hasattr(req.end_point, "strip") else req.end_point
        if not dest:
            self.get_logger().error("Empty end_point. Reject request.")
            if hasattr(resp, "success"):
                resp.success = False
            if hasattr(resp, "message"):
                resp.message = "empty end_point"
            return resp
        
        # ✅ 起點決策：如果使用者沒給 start_point，就用 TF+closest 算到的 current_origin_
        origin = req.start_point.strip() if hasattr(req.start_point, "strip") else req.start_point
        if not origin:
            origin = self.current_origin_

        if not origin:
            self.get_logger().error("No valid origin yet (TF/closest not ready). Reject request.")
            if hasattr(resp, "success"):
                resp.success = False
            if hasattr(resp, "message"):
                resp.message = "no origin: wait closest_service ready"
            return resp

        # 接受新目標：鎖起來 + 刷新 MPC
        self.allow_new_route_ = False
        self.nav_active_ = True

        self.goal_seq_ += 1
        self.latest_seq_ = self.goal_seq_

        self.call_enable_mpc(False)
        self.loadPath(origin, dest, seq=self.latest_seq_) 

        if hasattr(resp, "success"):
            resp.success = True
        if hasattr(resp, "message"):
            resp.message = f"accepted seq={self.latest_seq_}, origin={origin}, dest={dest}"
        return resp


def main(args=None):
    rclpy.init(args=args)
    d = Demo()
    rclpy.spin(d)
    rclpy.shutdown()

if __name__ == "__main__":
    main()