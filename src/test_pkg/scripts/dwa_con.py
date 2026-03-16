#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from campusrover_msgs.srv import PlannerFunction
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class DwaKeyboardTrigger(Node):
    def __init__(self):
        super().__init__('dwa_keyboard_trigger')
        
        # 建立服務客戶端
        self.client = self.create_client(PlannerFunction, 'planner_function_dwa')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 dwa_planner 服務中...')
            
        self.get_logger().info('DWA 觸發節點已就緒。')
        self.get_logger().info('請在終端機按下 [空白鍵] 來啟動 DWA Planner。')
        self.get_logger().info('按下 [Ctrl+C] 退出。')

    def send_dwa_request(self):
        req = PlannerFunction.Request()
        
        # 設定啟動參數
        req.action.data = True
        req.direction_inverse.data = False
        req.obstacle_avoidance.data = True
        
        # 設定模式：1 代表 MODE_GLOBAL_PATH 
        req.mode = PlannerFunction.Request.MODE_GLOBAL_PATH 
        
        # 設定速度參數
        speed = Twist()
        speed.linear.x = 0.5
        speed.angular.z = 0.3
        req.speed_parameter = speed

        self.get_logger().info('發送啟動請求：全域路徑模式, 避障開啟, 線速度 0.5, 角速度 0.3')
        
        # 非同步呼叫服務
        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('DWA Planner 已成功收到請求並啟動。')
        except Exception as e:
            self.get_logger().error(f'服務呼叫失敗: {e}')

def get_key(settings):
    """ 讀取單個字元輸入而不需按下 Enter """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = DwaKeyboardTrigger()

    try:
        while rclpy.ok():
            # 處理 ROS 回呼
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 檢查鍵盤輸入
            key = get_key(settings)
            if key == ' ':  # 空白鍵
                node.send_dwa_request()
            elif key == '\x03':  # Ctrl+C
                break
                
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()