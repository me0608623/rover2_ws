"""TrackedObstacleArray → 60D body-frame tensor (top-10 × 6D).

Replicates the training pipeline from IsaacLab's topk_obstacles_6d:
  Each obstacle: [dx_body, dy_body, vx_body, vy_body, radius, mask]
  - Positions/velocities are relative to robot, in robot body frame
  - Positions normalized by max_distance, velocities by v_max
  - Sorted by distance, top-K kept, rest zero-padded
  - mask = 1.0 if valid and within range, 0.0 otherwise

Training reference:
  IsaacLab/.../mdp/observations/obs_functions.py:topk_obstacles_6d
"""

import math
import numpy as np


def tracked_obstacles_to_60d(
    obstacles,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    robot_vx_w: float = 0.0,
    robot_vy_w: float = 0.0,
    top_k: int = 10,
    max_distance: float = 8.0,
    v_max: float = 1.5,
) -> np.ndarray:
    """Convert TrackedObstacleArray.obstacles list to 60D flat observation.

    Args:
        obstacles: list of TrackedObstacle messages
        robot_x, robot_y: robot position in map/odom frame
        robot_yaw: robot heading (rad)
        robot_vx_w, robot_vy_w: robot velocity in world frame
        top_k: number of obstacles to keep
        max_distance: max observation distance (m)
        v_max: velocity normalization factor (m/s)

    Returns:
        np.ndarray of shape (60,) = top_k * 6
    """
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)

    obstacle_features = []

    for obs in obstacles:
        # World frame position delta
        dx_w = obs.pose.position.x - robot_x
        dy_w = obs.pose.position.y - robot_y

        # Distance check
        dist = math.sqrt(dx_w * dx_w + dy_w * dy_w)
        if dist > max_distance or dist < 1e-6:
            continue

        # Rotate to body frame
        dx_body = cos_yaw * dx_w + sin_yaw * dy_w
        dy_body = -sin_yaw * dx_w + cos_yaw * dy_w

        # Relative velocity in world frame
        rel_vx_w = obs.velocity.linear.x - robot_vx_w
        rel_vy_w = obs.velocity.linear.y - robot_vy_w

        # Rotate velocity to body frame
        vx_body = cos_yaw * rel_vx_w + sin_yaw * rel_vy_w
        vy_body = -sin_yaw * rel_vx_w + cos_yaw * rel_vy_w

        # Normalize
        dx_norm = dx_body / max_distance
        dy_norm = dy_body / max_distance
        vx_norm = vx_body / v_max
        vy_norm = vy_body / v_max

        # Radius from dimensions
        radius = max(obs.dimensions.x, obs.dimensions.y) / 2.0
        if radius <= 0:
            radius = 0.3  # default

        # mask = 1.0 (valid obstacle)
        mask = 1.0

        obstacle_features.append({
            'dist': dist,
            'feat': [dx_norm, dy_norm, vx_norm, vy_norm, radius, mask],
        })

    # Sort by distance, take top-K
    obstacle_features.sort(key=lambda o: o['dist'])
    obstacle_features = obstacle_features[:top_k]

    # Build output array
    result = np.zeros(top_k * 6, dtype=np.float32)
    for i, obs_feat in enumerate(obstacle_features):
        result[i * 6:(i + 1) * 6] = obs_feat['feat']

    return result
