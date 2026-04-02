#!/usr/bin/env python3
"""RL Policy ROS2 Node — system Python 3.12 (compatible with rclpy jazzy).

Handles all ROS2 I/O. Delegates torch inference to a subprocess
(rl_inference_server.py) running with conda Python 3.11 (which has torch).

Communication protocol with inference subprocess:
  stdin → 139 × float32 = 556 bytes (observation)
  stdout ← 2 × int32    = 8 bytes  (accel_idx, omega_idx)
"""

import math
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2
from campusrover_msgs.msg import TrackedObstacleArray

# Add scripts dir to path for preprocessors
_SCRIPTS_DIR = Path(__file__).parent
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
from lidar_preprocessor import pointcloud2_to_72bins
from obstacle_preprocessor import tracked_obstacles_to_60d

OBS_DIM = 139
OBS_BYTES = OBS_DIM * 4   # float32 × 139
ACT_BYTES = 2 * 4          # int32 × 2

CONDA_PYTHON = '/home/aa/miniconda3/envs/env_isaaclab/bin/python'
INFERENCE_SERVER = str(_SCRIPTS_DIR / 'rl_inference_server.py')
CHECKPOINT_DEFAULT = (
    '/home/aa/IsaacLab/logs/skrl/'
    'Isaac-Navigation-Charge-VLP16-Curriculum-NavRL-AC/'
    'rw_groundv8_openendedv1__seed1_nowalls_diag_v9/'
    'checkpoints/agent_23436.pt'
)


class RLPolicyNode(Node):
    """RL policy inference node — replaces DWA planner."""

    NUM_BINS = 19
    CENTER = 9
    V_MAX = 1.0
    A_MAX = 0.5
    OMEGA_MAX = 0.25 * math.pi
    DT = 0.2

    MAX_ACCELERATION = 1.0
    MAX_LINEAR_VELOCITY = 1.0
    MAX_ANGULAR_VELOCITY = 1.5
    ROBOT_RADIUS = 0.35

    def __init__(self):
        super().__init__('rl_policy_node')

        # ── Parameters ──
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('deterministic', True)
        self.declare_parameter('use_cadn', True)
        self.declare_parameter('goal_timeout', 45.0)
        self.declare_parameter('goal_lookahead', 2.0)

        checkpoint_path = self.get_parameter('checkpoint_path').value or CHECKPOINT_DEFAULT
        device = self.get_parameter('device').value
        deterministic = self.get_parameter('deterministic').value
        use_cadn = self.get_parameter('use_cadn').value
        self.GOAL_TIMEOUT = self.get_parameter('goal_timeout').value
        self.GOAL_LOOKAHEAD = self.get_parameter('goal_lookahead').value

        # ── Launch inference subprocess ──
        self._proc = subprocess.Popen(
            [
                CONDA_PYTHON, INFERENCE_SERVER,
                '--checkpoint', checkpoint_path,
                '--device', device,
            ] + (['--deterministic'] if deterministic else [])
              + (['--no-cadn'] if not use_cadn else []),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,  # inherit stderr → printed to terminal
        )
        self.get_logger().info(
            f'Inference server started (pid={self._proc.pid})'
        )

        # ── State tracking ──
        self.current_v = 0.0
        self.prev_v = 0.0
        self.goal_start_time = None

        # ── Latest ROS data ──
        self.latest_odom = None
        self.latest_ndt_pose = None
        self.latest_lidar = None
        self.latest_obstacles = None
        self.latest_path = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, 'ndt_pose', self._ndt_pose_cb, 10)
        self.create_subscription(PointCloud2, 'velodyne_points', self._lidar_cb, sensor_qos)
        self.create_subscription(
            TrackedObstacleArray, 'tracked_label_obstacle', self._obstacle_cb, 10)
        self.create_subscription(NavPath, 'global_path', self._path_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self._goal_pose_cb, 10)
        self.latest_goal_pose = None

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(self.DT, self._inference_tick)

        self.get_logger().info('RL Policy Node ready')

    # ── Callbacks ──────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _ndt_pose_cb(self, msg):
        self.latest_ndt_pose = msg

    def _lidar_cb(self, msg):
        self.latest_lidar = msg

    def _obstacle_cb(self, msg):
        self.latest_obstacles = msg

    def _path_cb(self, msg):
        self.latest_path = msg
        self.goal_start_time = time.monotonic()

    def _goal_pose_cb(self, msg):
        self.latest_goal_pose = msg
        if self.goal_start_time is None:
            self.goal_start_time = time.monotonic()

    # ── Inference tick (5 Hz) ──────────────────────────────────────────

    def _inference_tick(self):
        if self.latest_odom is None or self.latest_lidar is None:
            return
        has_path = self.latest_path is not None and len(self.latest_path.poses) > 0
        has_goal = self.latest_goal_pose is not None
        if not has_path and not has_goal:
            return

        odom = self.latest_odom
        vx_body = odom.twist.twist.linear.x
        omega_z = odom.twist.twist.angular.z

        # Use ndt_pose (map frame) for position if available, else fall back to odom
        if self.latest_ndt_pose is not None:
            robot_x = self.latest_ndt_pose.pose.position.x
            robot_y = self.latest_ndt_pose.pose.position.y
            robot_yaw = self._quat_to_yaw(self.latest_ndt_pose.pose.orientation)
        else:
            robot_x = odom.pose.pose.position.x
            robot_y = odom.pose.pose.position.y
            robot_yaw = self._quat_to_yaw(odom.pose.pose.orientation)

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        robot_vx_w = cos_yaw * vx_body
        robot_vy_w = sin_yaw * vx_body

        # ── Ego state ──
        accel = (vx_body - self.prev_v) / self.DT
        self.prev_v = vx_body
        self.current_v = vx_body

        # NOTE: Training convention has NEGATED velocity and goal (forward = negative X).
        # We negate accel/velocity/goal to match training's sign convention.
        ego = np.array([
            float(np.clip(-accel / self.MAX_ACCELERATION, -1.0, 1.0)),
            float(np.clip(-vx_body / self.MAX_LINEAR_VELOCITY, -1.0, 1.0)),
            float(np.clip(omega_z / self.MAX_ANGULAR_VELOCITY, -1.0, 1.0)),
            self.ROBOT_RADIUS,
        ], dtype=np.float32)

        # ── Goal ──
        goal_body = -self._extract_goal(robot_x, robot_y, robot_yaw)

        # ── LiDAR ──
        lidar_bins = pointcloud2_to_72bins(self.latest_lidar).astype(np.float32)

        # ── Obstacles ──
        if self.latest_obstacles is not None and len(self.latest_obstacles.obstacles) > 0:
            obs_60d = tracked_obstacles_to_60d(
                self.latest_obstacles.obstacles,
                robot_x, robot_y, robot_yaw,
                robot_vx_w, robot_vy_w,
            )
        else:
            obs_60d = np.zeros(60, dtype=np.float32)

        # ── Time remaining ──
        if self.goal_start_time is not None:
            elapsed = time.monotonic() - self.goal_start_time
            time_remaining = float(max(0.0, 1.0 - elapsed / self.GOAL_TIMEOUT))
        else:
            time_remaining = 1.0

        # ── Assemble 139D obs ──
        obs = np.concatenate([
            ego,
            goal_body,
            lidar_bins,
            obs_60d,
            [time_remaining],
        ]).astype(np.float32)

        # ── Debug log (throttled) ──
        if not hasattr(self, '_dbg_cnt'):
            self._dbg_cnt = 0
        self._dbg_cnt += 1
        if self._dbg_cnt % 25 == 1:  # every 5 seconds at 5Hz
            bins_str = ','.join(f'{v:.3f}' for v in lidar_bins[:5]) + '...'
            self.get_logger().info(
                f'OBS ego={list(ego)} goal={list(goal_body)} '
                f'lidar_first5=[{bins_str}] '
                f'obs60_nonzero={np.count_nonzero(obs_60d)} time={time_remaining:.2f} '
                f'odom_vx={vx_body:.3f} current_v={self.current_v:.3f}'
            )

        # ── Send to subprocess, read action ──
        try:
            self._proc.stdin.write(obs.tobytes())
            self._proc.stdin.flush()

            action_bytes = b''
            while len(action_bytes) < ACT_BYTES:
                chunk = self._proc.stdout.read(ACT_BYTES - len(action_bytes))
                if not chunk:
                    self.get_logger().error('Inference server closed stdout!')
                    return
                action_bytes += chunk

            accel_idx, omega_idx = struct.unpack('<ii', action_bytes)
        except BrokenPipeError:
            self.get_logger().error('Inference server pipe broken!')
            return

        twist = self._decode_action(accel_idx, omega_idx)
        if self._dbg_cnt % 25 == 1:
            self.get_logger().info(
                f'ACT idx=({accel_idx},{omega_idx}) → v={twist.linear.x:.3f} ω={twist.angular.z:.3f}'
            )
        self.cmd_vel_pub.publish(twist)

    # ── Goal extraction ────────────────────────────────────────────────

    def _extract_goal(self, robot_x, robot_y, robot_yaw):
        # Direct goal_pose (map frame) — no path needed
        if (self.latest_path is None or len(self.latest_path.poses) == 0) \
                and self.latest_goal_pose is not None:
            gx = self.latest_goal_pose.pose.position.x
            gy = self.latest_goal_pose.pose.position.y
            dx_w = gx - robot_x
            dy_w = gy - robot_y
            cos_yaw = math.cos(robot_yaw)
            sin_yaw = math.sin(robot_yaw)
            return np.array([
                cos_yaw * dx_w + sin_yaw * dy_w,
                -sin_yaw * dx_w + cos_yaw * dy_w,
            ], dtype=np.float32)

        if self.latest_path is None or len(self.latest_path.poses) == 0:
            return np.zeros(2, dtype=np.float32)

        poses = self.latest_path.poses

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        for i, ps in enumerate(poses):
            dx = ps.pose.position.x - robot_x
            dy = ps.pose.position.y - robot_y
            d = math.sqrt(dx * dx + dy * dy)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # Walk forward by lookahead distance
        accumulated = 0.0
        goal_idx = closest_idx
        for i in range(closest_idx, len(poses) - 1):
            dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
            dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
            accumulated += math.sqrt(dx * dx + dy * dy)
            goal_idx = i + 1
            if accumulated >= self.GOAL_LOOKAHEAD:
                break

        gx = poses[goal_idx].pose.position.x
        gy = poses[goal_idx].pose.position.y
        dx_w = gx - robot_x
        dy_w = gy - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return np.array([
            cos_yaw * dx_w + sin_yaw * dy_w,
            -sin_yaw * dx_w + cos_yaw * dy_w,
        ], dtype=np.float32)

    # ── Action decoding ────────────────────────────────────────────────

    def _decode_action(self, accel_idx: int, omega_idx: int) -> Twist:
        ratio_linear = (accel_idx - self.CENTER) / self.CENTER
        ratio_angular = (omega_idx - self.CENTER) / self.CENTER

        # Decode in training convention (forward = negative velocity)
        v_train = -self.current_v  # convert ROS→training: negate
        allowable_max = min(self.A_MAX, (self.V_MAX - v_train) / self.DT)
        allowable_min = max(-self.A_MAX, (-self.V_MAX - v_train) / self.DT)

        if ratio_linear >= 0:
            accel = ratio_linear * allowable_max
        else:
            accel = -ratio_linear * allowable_min
        accel = max(allowable_min, min(allowable_max, accel))

        v_next_train = max(-self.V_MAX, min(self.V_MAX, v_train + accel * self.DT))
        omega = ratio_angular * self.OMEGA_MAX

        # Convert back to ROS convention
        v_next_ros = -v_next_train
        self.current_v = v_next_ros

        twist = Twist()
        twist.linear.x = v_next_ros
        twist.angular.z = omega
        return twist

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def destroy_node(self):
        self._proc.terminate()
        self._proc.wait(timeout=3)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
