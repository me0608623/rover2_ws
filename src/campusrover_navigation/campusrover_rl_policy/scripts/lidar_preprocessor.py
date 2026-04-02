"""LiDAR PointCloud2 → 72-bin 2D distance vector.

Replicates the training pipeline from IsaacLab's lidar_vlp16_to_2d_bins:
  1. Parse PointCloud2 → Nx3 numpy array (x, y, z)
  2. Project to 2D: horizontal distance = sqrt(x^2 + y^2)
  3. Compute azimuth: atan2(y, x) → [0, 2*pi]
  4. Bin into 72 azimuthal bins (5 deg each), min distance per bin
  5. Subtract r_robot (0.3m), clamp >= 0
  6. Normalize by r_max (20.0m) → [0, 1]

Training reference:
  IsaacLab/.../mdp/observations/obs_functions.py:lidar_vlp16_to_2d_bins
"""

import math
import struct

import numpy as np


def parse_pointcloud2(msg) -> np.ndarray:
    """Parse sensor_msgs/PointCloud2 to Nx3 float32 array (x, y, z).

    Handles both structured (with fields) and raw point clouds.
    Assumes 'x', 'y', 'z' fields exist at known offsets.
    """
    # Fast path: read raw bytes
    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if len(data) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Find x, y, z field offsets
    field_offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            field_offsets[field.name] = field.offset

    if len(field_offsets) < 3:
        # Fallback: assume x=0, y=4, z=8 (standard layout)
        field_offsets = {'x': 0, 'y': 4, 'z': 8}

    n_points = msg.width * msg.height
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Reshape and extract float32 fields
    points = np.zeros((n_points, 3), dtype=np.float32)
    raw = data.reshape(n_points, point_step)

    for i, axis in enumerate(('x', 'y', 'z')):
        offset = field_offsets[axis]
        points[:, i] = np.frombuffer(
            raw[:, offset:offset + 4].tobytes(), dtype=np.float32
        )

    # Filter out NaN and inf
    valid = np.isfinite(points).all(axis=1)
    return points[valid]


def pointcloud2_to_72bins(
    msg,
    num_bins: int = 72,
    r_max: float = 20.0,
    r_robot: float = 0.35,
) -> np.ndarray:
    """Convert PointCloud2 to 72-bin normalized distance vector.

    Args:
        msg: sensor_msgs/PointCloud2 message
        num_bins: number of azimuthal bins (default 72 = 5 deg each)
        r_max: maximum detection range (m)
        r_robot: robot body radius to subtract (m)

    Returns:
        np.ndarray of shape (num_bins,), values in [0, 1]
    """
    points = parse_pointcloud2(msg)

    if len(points) == 0:
        # No data → all bins at max range (normalized to 1.0 after robot subtraction)
        return np.ones(num_bins, dtype=np.float32) * ((r_max - r_robot) / r_max)

    # Filter by height: remove floor/ceiling hits
    # VLP-16 mounted at ~0.5m, keep points between -0.3m and 1.5m relative to sensor
    z = points[:, 2]
    height_mask = (z > -0.3) & (z < 1.5)
    points = points[height_mask]

    if len(points) == 0:
        return np.ones(num_bins, dtype=np.float32) * ((r_max - r_robot) / r_max)

    # 2D horizontal distance
    x, y = points[:, 0], points[:, 1]
    dist_2d = np.sqrt(x * x + y * y)

    # Filter out robot self-hits (training only raycasts ground/walls/obstacles, not robot body)
    far_enough = dist_2d > 0.5
    x, y, dist_2d = x[far_enough], y[far_enough], dist_2d[far_enough]

    if len(dist_2d) == 0:
        return np.ones(num_bins, dtype=np.float32) * ((r_max - r_robot) / r_max)

    # Clamp distance
    dist_2d = np.clip(dist_2d, 0.0, r_max)

    # Azimuth angle [0, 2*pi)
    # atan2(y, x) gives [-pi, pi], shift to [0, 2*pi)
    azimuth = np.arctan2(y, x)
    azimuth = azimuth % (2.0 * math.pi)

    # Bin assignment: bin_k covers [k * bin_width, (k+1) * bin_width)
    bin_width = 2.0 * math.pi / num_bins
    bin_idx = np.floor(azimuth / bin_width).astype(np.int32)
    bin_idx = np.clip(bin_idx, 0, num_bins - 1)

    # Min distance per bin (init to r_max)
    bins = np.full(num_bins, r_max, dtype=np.float32)
    for i in range(num_bins):
        mask = bin_idx == i
        if mask.any():
            bins[i] = dist_2d[mask].min()

    # Vectorized alternative (faster for large point clouds):
    # Use np.minimum.at for scatter-min
    bins_fast = np.full(num_bins, r_max, dtype=np.float32)
    np.minimum.at(bins_fast, bin_idx, dist_2d.astype(np.float32))
    bins = bins_fast

    # Safe margin: subtract robot radius, clamp >= 0
    bins = np.maximum(bins - r_robot, 0.0)

    # Normalize to [0, 1]
    bins = bins / r_max

    # Reorder: our bin 0 = forward (0°), training bin 0 = back (-180°)
    # Training order: -180°, -175°, ..., 0°(bin36), ..., +175°(bin71)
    # Our order:      0°(bin0), 5°, ..., 180°(bin36), ..., 355°(bin71)
    # Shift: training_bins = roll(our_bins, -36)
    bins = np.roll(bins, -36)

    return bins
