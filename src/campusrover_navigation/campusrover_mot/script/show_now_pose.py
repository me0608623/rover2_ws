#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def main(args=None):
    rclpy.init(args=args)
    node = Node('show_now_pose')

    robot_frame = 'base_link'
    map_frame = 'map'

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    now_pose_ = []
    node.get_logger().info('start recording ...')

    while len(now_pose_) < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            trans = tf_buffer.lookup_transform(map_frame, robot_frame, rclpy.time.Time())
            now_pose_.append(trans)
            print(len(now_pose_))
        except Exception:
            pass
        time.sleep(0.1)

    node.get_logger().info('end recording ...')
    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    for item in now_pose_:
        t = item.transform.translation
        r = item.transform.rotation
        x += t.x
        y += t.y
        z += t.z

        e = quaternion_to_euler(r.x, r.y, r.z, r.w)
        roll += e[0]
        pitch += e[1]
        yaw += e[2]

    x /= 10
    y /= 10
    z /= 10

    q = euler_to_quaternion(roll / 10, pitch / 10, yaw / 10)

    print(x, ',', y, ',', z, ',', q[0], ',', q[1], ',', q[2], ',', q[3])

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
