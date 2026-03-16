#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Path

class BubbleBridgeNode(Node):
    def __init__(self):
        super().__init__('bubble_bridge_node')
        
        # 狀態位元：True 表示正常輸出，False 表示中斷輸出
        self.is_active = True
        
        # 1. 建立服務 (處理中斷/觸發邏輯)
        # 注意：您提到的 std_msgs/msg/Triger 應為 std_srvs/srv/Trigger
        self.srv = self.create_service(Trigger, 'stop_bubble', self.toggle_bridge_callback)
        
        # 2. 建立訂閱者 (接收原始資料)
        self.info_sub = self.create_subscription(Path, 'bubble_info', self.info_callback, 10)
        self.path_sub = self.create_subscription(Path, 'bubble_path', self.path_callback, 10)
        
        # 3. 建立發布者 (測試用輸出)
        self.info_pub = self.create_publisher(Path, 'test_bubble_info', 10)
        self.path_pub = self.create_publisher(Path, 'test_bubble_path', 10)
        
        self.get_logger().info("Bubble Bridge Node 已啟動，目前狀態：轉發中")

    def toggle_bridge_callback(self, request, response):
        # 切換狀態
        self.is_active = not self.is_active
        status_str = "轉發中" if self.is_active else "已中斷"
        
        response.success = True
        response.message = f"目前 Bubble 輸出狀態：{status_str}"
        self.get_logger().info(response.message)
        return response

    def info_callback(self, msg):
        if self.is_active:
            self.info_pub.publish(msg)

    def path_callback(self, msg):
        if self.is_active:
            self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BubbleBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()