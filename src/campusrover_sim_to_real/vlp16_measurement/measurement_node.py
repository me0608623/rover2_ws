#!/usr/bin/env python3
"""
VLP-16 LiDAR 靜態距離量測節點

訂閱 VLP-16 點雲，對指定方向的標靶進行多幀採樣，
計算每幀的中位數水平距離，收集完畢後輸出統計結果與 JSON 檔。

使用方式：
    python3 measurement_node.py --distance 2.0 --n_samples 100 --angle_range 5.0 \
        --height_range 0.3 --output_dir ./data --material white_wall
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stats_utils import compute_statistics, format_statistics_report

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


def parse_args():
    """解析命令列參數，並分離 ROS2 專用參數。"""
    parser = argparse.ArgumentParser(
        description="VLP-16 LiDAR 靜態距離量測節點",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "範例：\n"
            "  python3 measurement_node.py --distance 2.0 --n_samples 100\n"
            "  python3 measurement_node.py --distance 5.0 --material black_board\n"
        ),
    )
    parser.add_argument(
        "--distance", type=float, required=True,
        help="Ground Truth 距離（單位：m，由雷射測距儀量測後手動輸入）",
    )
    parser.add_argument(
        "--n_samples", type=int, default=100,
        help="採樣幀數（預設：100）",
    )
    parser.add_argument(
        "--angle_range", type=float, default=5.0,
        help="水平篩選角度 ±度數（預設：5.0）",
    )
    parser.add_argument(
        "--height_range", type=float, default=0.3,
        help="z 軸篩選範圍 ±公尺（預設：0.3）",
    )
    parser.add_argument(
        "--output_dir", type=str, default="./data",
        help="JSON 輸出目錄（預設：./data）",
    )
    parser.add_argument(
        "--topic", type=str, default="/velodyne_points",
        help="點雲 topic（預設：/velodyne_points）",
    )
    parser.add_argument(
        "--material", type=str, default="white_wall",
        choices=["white_wall", "black_board", "human"],
        help="標靶材質標籤���預設：white_wall）",
    )

    args, ros_args = parser.parse_known_args()
    return args, ros_args


class VLP16MeasurementNode(Node):
    """VLP-16 靜態距離量測 ROS2 節點。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("vlp16_measurement_node")

        self.distance = args.distance
        self.n_samples = args.n_samples
        self.angle_range = args.angle_range
        self.height_range = args.height_range
        self.output_dir = args.output_dir
        self.topic = args.topic
        self.material = args.material

        # 預計算篩選常數（避免每幀重算）
        self._dist_lo_sq = (self.distance * 0.5) ** 2
        self._dist_hi_sq = (self.distance * 2.0) ** 2

        self.samples: list = []
        self.frame_count: int = 0
        self.consecutive_empty: int = 0
        self.start_time: float = time.time()
        self.first_msg_received: bool = False
        self.finished: bool = False

        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f"無法建立輸出目錄 {self.output_dir}: {e}")
            raise SystemExit(1)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription = self.create_subscription(
            PointCloud2, self.topic, self._pointcloud_callback, qos
        )

        self.watchdog_timer = self.create_timer(10.0, self._watchdog_callback)

        print("\n" + "=" * 60)
        print("  VLP-16 靜態距離量測節點 已啟動")
        print("=" * 60)
        print(f"  Ground Truth 距離 : {self.distance:.3f} m")
        print(f"  目標採樣幀數      : {self.n_samples}")
        print(f"  方位角篩選範圍    : ±{self.angle_range:.1f}°")
        print(f"  高度篩選範圍      : ±{self.height_range:.2f} m")
        print(f"  標靶材質          : {self.material}")
        print(f"  點雲 Topic        : {self.topic}")
        print(f"  輸出目錄          : {os.path.abspath(self.output_dir)}")
        print("=" * 60)
        print("  等待點雲訊息中...\n")

    def _watchdog_callback(self):
        """定時檢查是否收到點雲訊息。"""
        if self.finished:
            return
        elapsed = time.time() - self.start_time
        if not self.first_msg_received and elapsed > 30.0:
            print(
                f"[警告] 已等待 {elapsed:.0f} 秒仍未收到任何點雲訊息。\n"
                f"       請確認 topic '{self.topic}' 是否正確，VLP-16 是否已啟動。\n"
                f"       可用 'ros2 topic list' 檢查���用的 topic。"
            )

    def _pointcloud_callback(self, msg: PointCloud2):
        """處理每一幀點雲訊息（分階段篩選以減少不必要的計算）。"""
        if self.finished:
            return

        if not self.first_msg_received:
            self.first_msg_received = True
            print("[資訊] 已收到第一幀點雲，開始採樣...\n")

        self.frame_count += 1

        try:
            points = read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
            x = np.array(points["x"], dtype=np.float32)
            y = np.array(points["y"], dtype=np.float32)
            z = np.array(points["z"], dtype=np.float32)
        except Exception as e:
            self.get_logger().warning(f"點雲解析失敗（第 {self.frame_count} 幀）: {e}")
            return

        if len(x) == 0:
            self.consecutive_empty += 1
            self._check_consecutive_empty()
            return

        # 分階段篩選：先做最便宜的 height filter
        mask_h = np.abs(z) <= self.height_range
        x_h, y_h = x[mask_h], y[mask_h]

        if len(x_h) < 3:
            self.consecutive_empty += 1
            self._check_consecutive_empty()
            return

        # 用平方距離篩選（避免 sqrt）
        horiz_dist_sq = x_h * x_h + y_h * y_h
        mask_d = (horiz_dist_sq >= self._dist_lo_sq) & (horiz_dist_sq <= self._dist_hi_sq)
        x_d, y_d = x_h[mask_d], y_h[mask_d]
        horiz_dist_sq_d = horiz_dist_sq[mask_d]

        if len(x_d) < 3:
            self.consecutive_empty += 1
            self._check_consecutive_empty()
            return

        # arctan2 只對存活點計算（最昂貴的操作）
        azimuth_deg = np.degrees(np.arctan2(y_d, x_d))
        mask_a = np.abs(azimuth_deg) <= self.angle_range

        if np.sum(mask_a) < 3:
            self.consecutive_empty += 1
            self._check_consecutive_empty()
            return

        # sqrt 只對最終有效點計算
        valid_dist = np.sqrt(horiz_dist_sq_d[mask_a])

        frame_median = float(np.median(valid_dist))
        self.samples.append(frame_median)
        self.consecutive_empty = 0

        n_collected = len(self.samples)
        if n_collected % 10 == 0 or n_collected == 1:
            print(
                f"[進度] 已收集 {n_collected}/{self.n_samples} 幀，"
                f"當前中位數: {frame_median:.3f}m，GT: {self.distance:.3f}m"
            )

        if n_collected >= self.n_samples:
            self._finish_collection()

    def _check_consecutive_empty(self):
        """檢查連續空幀數並發出警告。"""
        if self.consecutive_empty >= 20 and self.consecutive_empty % 20 == 0:
            print(
                f"\n[警告] 連續 {self.consecutive_empty} 幀無有效點！\n"
                f"       請確認：\n"
                f"       1. LiDAR 是否正對標靶\n"
                f"       2. 方位角範圍 ±{self.angle_range}° 是否合適\n"
                f"       3. 標靶是否在 {self.distance * 0.5:.1f}m ~ {self.distance * 2.0:.1f}m 範圍內\n"
                f"       4. 高度範圍 ±{self.height_range:.2f}m 是否涵蓋標靶\n"
            )

    # ------------------------------------------------------------------
    # JSON 組裝與儲存（共用邏輯）
    # ------------------------------------------------------------------

    def _build_result_dict(self) -> tuple:
        """組裝量測結果字典，回傳 (result_dict, stats_dict)。"""
        now = datetime.now()
        stats = compute_statistics(self.samples, self.distance)
        result = {
            "metadata": {
                "ground_truth_m": self.distance,
                "material": self.material,
                "timestamp": now.isoformat(),
                "n_requested": self.n_samples,
                "n_valid_frames": len(self.samples),
                "total_frames_processed": self.frame_count,
                "angle_range_deg": self.angle_range,
                "height_range_m": self.height_range,
                "topic": self.topic,
            },
            "statistics": stats,
            "raw_samples": [float(s) for s in self.samples],
        }
        return result, stats, now

    def _save_json(self, suffix: str = "") -> str | None:
        """儲存 JSON 檔案，回傳檔案路徑。"""
        result, stats, now = self._build_result_dict()
        ts_short = now.strftime("%Y%m%d_%H%M%S")
        filename = f"measurement_{self.distance}m_{self.material}_{ts_short}{suffix}.json"
        filepath = os.path.join(self.output_dir, filename)

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            return filepath
        except OSError as e:
            print(f"[錯誤] 無法儲存 JSON 檔案：{e}")
            return None

    def _finish_collection(self):
        """採樣完成，計算統計、儲存 JSON、印出結果。"""
        self.finished = True
        collection_time = time.time() - self.start_time

        stats = compute_statistics(self.samples, self.distance)
        report = format_statistics_report(stats, self.distance, len(self.samples))
        print("\n" + report)

        filepath = self._save_json()
        if filepath:
            print(f"\n[儲存] 量測結果已儲存至：{os.path.abspath(filepath)}")

        print(f"\n  採樣耗時：{collection_time:.1f} 秒")
        print("\n" + "-" * 60)
        print("  下一步操作：")
        print("  1. 將標靶移動到下一個距離位置")
        print("  2. 用雷射測距儀量測新的 Ground Truth 距離")
        print("  3. 再次執行本程式，例如：")
        print(f"     python3 measurement_node.py --distance <新距離> "
              f"--material {self.material} --output_dir {self.output_dir}")
        print("-" * 60)
        print("  所有距離量測完成後，執行分析腳本：")
        print(f"     python3 analyze_gap.py --data_dir {self.output_dir}")
        print("-" * 60 + "\n")

        raise SystemExit(0)

    def save_partial_results(self):
        """Ctrl+C 中斷時，儲存已收集的部分結果。"""
        if not self.samples or self.finished:
            return

        print(f"\n[中斷] 收到中斷信號，已收集 {len(self.samples)}/{self.n_samples} 幀")

        stats = compute_statistics(self.samples, self.distance)
        report = format_statistics_report(stats, self.distance, len(self.samples))
        print(report)

        filepath = self._save_json(suffix="_partial")
        if filepath:
            print(f"[儲存] 部分結果已儲存至：{os.path.abspath(filepath)}")


def main():
    args, ros_args = parse_args()

    rclpy.init(args=ros_args)
    node = VLP16MeasurementNode(args)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.save_partial_results()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
