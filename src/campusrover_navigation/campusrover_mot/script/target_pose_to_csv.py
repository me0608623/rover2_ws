#!/usr/bin/env python3
import pandas as pd

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__('target_pose')
        self.pd_data = pd.DataFrame()

        self.robot_frame = 'base_link'
        self.map_frame = 'map'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())

            stamp_sec = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9

            new_data = pd.DataFrame({
                'time': [stamp_sec],
                'pos_x': [trans.transform.translation.x],
                'pos_y': [trans.transform.translation.y],
            })

            self.pd_data = pd.concat([self.pd_data, new_data], ignore_index=True)
        except Exception as ex:
            print(ex)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        now_sec = node.get_clock().now().nanoseconds * 1e-9
        node.pd_data.to_csv(f'~/target_pose_{now_sec}.csv', index=None)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
