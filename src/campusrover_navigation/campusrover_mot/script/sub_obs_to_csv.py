#!/usr/bin/env python3
import pandas as pd

import rclpy
from rclpy.node import Node
from campusrover_msgs.msg import TrackedObstacleArray


class SubObsNode(Node):
    def __init__(self):
        super().__init__('sub_obs')
        self.pd_data = pd.DataFrame()
        self.create_subscription(TrackedObstacleArray, 'tracked_label_obstacle', self.sub_callback, 10)

    def sub_callback(self, data):
        stamp_sec = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

        for item in data.obstacles:
            new_data = pd.DataFrame({
                'time': [stamp_sec],
                'ID': [item.id],
                'pos_x': [item.pose.position.x],
                'pos_y': [item.pose.position.y],
                'vel_x': [item.velocity.linear.x],
                'vel_y': [item.velocity.linear.y],
            })

            self.pd_data = pd.concat([self.pd_data, new_data], ignore_index=True)


def main(args=None):
    rclpy.init(args=args)
    node = SubObsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        now_sec = node.get_clock().now().nanoseconds * 1e-9
        node.pd_data.to_csv(f'~/mot_exp/output_all{now_sec}.csv', index=None)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
