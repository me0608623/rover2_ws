#!/usr/bin/env python3
"""
MOT Marker Visualization Node
Subscribes to TrackedObstacleArray and publishes MarkerArray for RViz.
Replaces the ROS1 campusrover_msgs_visualization package.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from campusrover_msgs.msg import TrackedObstacleArray

# Label colors: (r, g, b)
LABEL_COLORS = {
    0: (1.0, 1.0, 1.0),   # UNKNOWN: white
    1: (0.0, 1.0, 0.0),   # PERSON: green
    2: (0.0, 0.5, 1.0),   # BICYCLE: blue
    3: (1.0, 0.0, 0.0),   # CAR: red
    4: (1.0, 1.0, 0.0),   # MOTORCYCLE: yellow
}

LABEL_NAMES = {
    0: 'unknown',
    1: 'person',
    2: 'bicycle',
    3: 'car',
    4: 'motorcycle',
}


class MotMarkerNode(Node):
    def __init__(self):
        super().__init__('mot_marker_node')
        self.sub = self.create_subscription(
            TrackedObstacleArray,
            'tracked_obstacles',
            self.callback,
            10
        )
        self.pub = self.create_publisher(MarkerArray, 'obstacles_marker', 10)
        self.prev_count = 0
        self.get_logger().info('mot_marker_node started')

    def callback(self, msg: TrackedObstacleArray):
        ma = MarkerArray()
        frame_id = msg.header.frame_id or 'map'
        stamp = msg.header.stamp

        for i, obs in enumerate(msg.obstacles):
            r, g, b = LABEL_COLORS.get(obs.label, (1.0, 1.0, 1.0))

            # Bounding box (CUBE)
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = 'mot_box'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = obs.pose
            dim = max(obs.dimensions.x, 0.3)
            m.scale.x = dim
            m.scale.y = dim
            m.scale.z = max(obs.dimensions.z, 0.6) if obs.dimensions.z > 0.01 else 0.6
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.4
            m.lifetime.sec = 0
            m.lifetime.nanosec = 500000000  # 0.5s
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header.frame_id = frame_id
            t.header.stamp = stamp
            t.ns = 'mot_text'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = obs.pose.position.x
            t.pose.position.y = obs.pose.position.y
            t.pose.position.z = obs.pose.position.z + m.scale.z / 2.0 + 0.3
            label_name = LABEL_NAMES.get(obs.label, f'L{obs.label}')
            speed = (obs.velocity.linear.x ** 2 + obs.velocity.linear.y ** 2) ** 0.5
            dyn_tag = 'D' if obs.is_dynamic else 'S'
            t.text = f'[{obs.id}] {label_name} {dyn_tag} {speed:.1f}m/s'
            t.scale.z = 0.3
            t.color.r = r
            t.color.g = g
            t.color.b = b
            t.color.a = 1.0
            t.lifetime.sec = 0
            t.lifetime.nanosec = 500000000
            ma.markers.append(t)

            # Velocity arrow (only if moving)
            if speed > 0.05:
                a = Marker()
                a.header.frame_id = frame_id
                a.header.stamp = stamp
                a.ns = 'mot_vel'
                a.id = i
                a.type = Marker.ARROW
                a.action = Marker.ADD
                a.pose.position = obs.pose.position
                # Arrow direction from velocity
                import math
                yaw = math.atan2(obs.velocity.linear.y, obs.velocity.linear.x)
                a.pose.orientation.z = math.sin(yaw / 2.0)
                a.pose.orientation.w = math.cos(yaw / 2.0)
                a.scale.x = min(speed, 3.0)  # arrow length capped at 3m
                a.scale.y = 0.08
                a.scale.z = 0.08
                a.color.r = 1.0
                a.color.g = 0.5
                a.color.b = 0.0
                a.color.a = 0.9
                a.lifetime.sec = 0
                a.lifetime.nanosec = 500000000
                ma.markers.append(a)

        # Delete old markers if count decreased
        cur_count = len(msg.obstacles)
        if cur_count < self.prev_count:
            for j in range(cur_count, self.prev_count):
                for ns in ('mot_box', 'mot_text', 'mot_vel'):
                    d = Marker()
                    d.header.frame_id = frame_id
                    d.header.stamp = stamp
                    d.ns = ns
                    d.id = j
                    d.action = Marker.DELETE
                    ma.markers.append(d)
        self.prev_count = cur_count

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = MotMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
