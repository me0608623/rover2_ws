#!/usr/bin/env python3
"""
VLP-16 LiDAR Sim-to-Real 測距工具 — 互動式終端介面

提供系統化的操作流程，引導使用者完成：
  1. 單次距離量測
  2. 批次多距離量測
  3. 資料分析與報告產生

使用方式：
    python3 vlp16_tool.py
"""

import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MEASUREMENT_SCRIPT = os.path.join(SCRIPT_DIR, "measurement_node.py")
ANALYZE_SCRIPT = os.path.join(SCRIPT_DIR, "analyze_gap.py")

MATERIALS = ["white_wall", "black_board", "human"]


def clear_screen():
    """清除終端畫面。"""
    os.system("clear" if os.name != "nt" else "cls")


def print_header():
    """印出主選單標題。"""
    print("\n" + "=" * 56)
    print("  VLP-16 LiDAR Sim-to-Real 測距工具")
    print("=" * 56)


def input_with_default(prompt: str, default: str) -> str:
    """帶預設值的輸入，使用者直接 Enter 則採用預設值。"""
    result = input(f"  {prompt} [{default}]: ").strip()
    return result if result else default


def input_float(prompt: str, default: float) -> float:
    """帶預設值的浮點數輸入，含驗證。"""
    while True:
        raw = input(f"  {prompt} [{default}]: ").strip()
        if not raw:
            return default
        try:
            val = float(raw)
            if val <= 0:
                print("  [錯誤] 請輸入正數")
                continue
            return val
        except ValueError:
            print("  [錯誤] 請輸入有效的數字")


def input_int(prompt: str, default: int) -> int:
    """帶預設值的整數輸入，含驗證。"""
    while True:
        raw = input(f"  {prompt} [{default}]: ").strip()
        if not raw:
            return default
        try:
            val = int(raw)
            if val <= 0:
                print("  [錯誤] 請輸入正整數")
                continue
            return val
        except ValueError:
            print("  [錯誤] 請輸入有效的整數")


def input_material() -> str:
    """選擇標靶材質。"""
    print("\n  標靶材質選擇：")
    for i, m in enumerate(MATERIALS, 1):
        print(f"    [{i}] {m}")
    while True:
        raw = input(f"  請選擇 [1]: ").strip()
        if not raw:
            return MATERIALS[0]
        try:
            idx = int(raw)
            if 1 <= idx <= len(MATERIALS):
                return MATERIALS[idx - 1]
        except ValueError:
            pass
        print("  [錯誤] 請輸入有效的選項編號")


def confirm(prompt: str) -> bool:
    """確認提示，預設為 yes。"""
    raw = input(f"  {prompt} [Y/n]: ").strip().lower()
    return raw != "n"


def run_measurement(distance: float, material: str, n_samples: int,
                    angle_range: float, height_range: float,
                    output_dir: str, topic: str) -> int:
    """執行單次量測（透過 subprocess 隔離 ROS2 生命週期）。"""
    cmd = [
        sys.executable, MEASUREMENT_SCRIPT,
        "--distance", str(distance),
        "--n_samples", str(n_samples),
        "--angle_range", str(angle_range),
        "--height_range", str(height_range),
        "--output_dir", output_dir,
        "--topic", topic,
        "--material", material,
    ]
    print(f"\n  即將執行：python3 measurement_node.py --distance {distance} "
          f"--n_samples {n_samples} --material {material}")
    print("  按 Ctrl+C 可中斷量測\n")

    try:
        result = subprocess.run(cmd)
        return result.returncode
    except KeyboardInterrupt:
        print("\n  [中斷] 使用者中斷量測")
        return 1


def run_analysis(data_dir: str, output_dir: str, material: str,
                 min_samples: int) -> int:
    """執行分析腳本。"""
    cmd = [
        sys.executable, ANALYZE_SCRIPT,
        "--data_dir", data_dir,
        "--output_dir", output_dir,
        "--material", material,
        "--min_samples", str(min_samples),
    ]
    print(f"\n  即將執行：python3 analyze_gap.py --data_dir {data_dir} "
          f"--material {material}\n")

    try:
        result = subprocess.run(cmd)
        return result.returncode
    except KeyboardInterrupt:
        print("\n  [中斷] 使用者中斷分析")
        return 1


# ---------------------------------------------------------------------------
# 功能模組
# ---------------------------------------------------------------------------

def menu_single_measurement():
    """[1] 單次距離量測。"""
    print("\n" + "-" * 56)
    print("  單次距離量測")
    print("-" * 56)

    distance = input_float("Ground Truth 距離 (m)", 2.0)
    material = input_material()
    n_samples = input_int("採樣幀數", 100)
    angle_range = input_float("方位角篩選 ±度數", 5.0)
    height_range = input_float("高度篩選 ±公尺", 0.3)
    output_dir = input_with_default("輸出目錄", "./data")
    topic = input_with_default("點雲 Topic", "/velodyne_points")

    print("\n" + "-" * 56)
    print("  量測參數確認：")
    print(f"    距離: {distance}m | 材質: {material} | 幀數: {n_samples}")
    print(f"    角度: ±{angle_range}° | 高度: ±{height_range}m")
    print(f"    輸出: {output_dir} | Topic: {topic}")
    print("-" * 56)

    if not confirm("確認開始量測？"):
        print("  已取消。")
        return

    run_measurement(distance, material, n_samples, angle_range,
                    height_range, output_dir, topic)


def menu_batch_measurement():
    """[2] 批次多距離量測（引導式）。"""
    print("\n" + "-" * 56)
    print("  批次多距離量測")
    print("-" * 56)
    print("  本模式會引導你逐一量測多個距離。")
    print("  每個距離量測完成後，系統會提示你移動標靶。\n")

    n_distances = input_int("共幾個距離要量測", 5)
    material = input_material()
    n_samples = input_int("每個距離的採樣幀數", 100)
    angle_range = input_float("方位角篩選 ±度數", 5.0)
    height_range = input_float("高度篩選 ±公尺", 0.3)
    output_dir = input_with_default("輸出目錄", "./data")
    topic = input_with_default("點雲 Topic", "/velodyne_points")

    print(f"\n  共 {n_distances} 個距離，材質: {material}，每次 {n_samples} 幀")

    for i in range(1, n_distances + 1):
        print("\n" + "=" * 56)
        print(f"  第 {i}/{n_distances} 個距離")
        print("=" * 56)

        if i > 1:
            print("  請將標靶移動到下一個距離位置。")
            input("  準備好後按 Enter 繼續...")

        distance = input_float(f"第 {i} 個 Ground Truth 距離 (m)", float(i))

        returncode = run_measurement(distance, material, n_samples,
                                     angle_range, height_range,
                                     output_dir, topic)

        if returncode != 0:
            print(f"\n  [警告] 第 {i} 個距離量測未正常完成（返回碼: {returncode}）")
            if not confirm("是否繼續下一個距離？"):
                break

    print("\n" + "=" * 56)
    print("  批次量測完成！")
    print("=" * 56)

    if confirm("是否立即執行分析？"):
        run_analysis(output_dir, "./results", material, 1)


def menu_analyze():
    """[3] 分析已收集的資料。"""
    print("\n" + "-" * 56)
    print("  分析已收集的資料")
    print("-" * 56)

    data_dir = input_with_default("JSON 資料目錄", "./data")

    # 檢查目錄是否存在
    if not os.path.isdir(data_dir):
        print(f"  [錯誤] 目錄不存在：{os.path.abspath(data_dir)}")
        return

    # 列出可用的 JSON 檔案
    import glob
    files = glob.glob(os.path.join(data_dir, "measurement_*.json"))
    if not files:
        print(f"  [錯誤] 在 {data_dir} 中找不到任何 measurement_*.json 檔案")
        return

    print(f"  找到 {len(files)} 個量測檔案")

    material = input_with_default("篩選材質（all=不篩選）", "all")
    output_dir = input_with_default("結果輸出目錄", "./results")
    min_samples = input_int("每距離最少量測數", 1)

    print(f"\n  資料目錄: {data_dir} | 材質: {material} | 輸出: {output_dir}")

    if not confirm("確認開始分析？"):
        print("  已取消。")
        return

    run_analysis(data_dir, output_dir, material, min_samples)


def menu_help():
    """[4] 使用說明。"""
    print("""
========================================================
  VLP-16 LiDAR 測距工具 — 使用說明
========================================================

  本工具用於量測 VLP-16 LiDAR 的距離偏差特性，
  為 Sim-to-Real 的噪聲模型提供實測數據。

  ┌─────────────────────────────────────────────────┐
  │  完整量測流程                                    │
  ├─────────────────────────────────────────────────┤
  │  1. 將機器人固定，朝向平坦標靶                    │
  │  2. 用雷射測距儀量測 LiDAR 到標靶的距離            │
  │  3. 執行「單次量測」或「批次量測」                  │
  │  4. 移動標靶到不同距離，重複步驟 2-3              │
  │  5. 建議量測 5+ 個距離（如 1m, 2m, 3m, 5m, 8m）  │
  │  6. 執行「分析」產生偏差報告與補償參數              │
  └─────────────────────────────────────────────────┘

  標靶擺放注意事項：
    - 標靶應正對 LiDAR 前方（x 軸方向）
    - 標靶面積需大於 30cm x 30cm
    - 避免反光材質（玻璃、鏡面金屬）
    - 確保標靶在 LiDAR 視野的中心區域

  輸出檔案說明：
    data/measurement_*.json  — 原始量測數據
    results/bias_analysis.png     — 偏差分析圖
    results/noise_analysis.png    — 噪聲分析圖
    results/error_distribution.png — 誤差分佈圖
    results/isaac_lab_noise_params.py — Isaac Lab 參數
    results/gap_analysis_report.md — 完整分析報告

  建議距離範圍：
    VLP-16 有效範圍 0.5m ~ 100m
    建議量測 1m ~ 10m 之間的 5-8 個距離

========================================================
""")
    input("  按 Enter 返回主選單...")


# ---------------------------------------------------------------------------
# 主程式
# ---------------------------------------------------------------------------

def main():
    while True:
        print_header()
        print("  請選擇功能：")
        print("    [1] 單次距離量測")
        print("    [2] 批次多距離量測（引導式）")
        print("    [3] 分析已收集的資料")
        print("    [4] 說明 / 使用教學")
        print("    [0] 離開")
        print("=" * 56)

        choice = input("  請輸入選項: ").strip()

        if choice == "1":
            menu_single_measurement()
        elif choice == "2":
            menu_batch_measurement()
        elif choice == "3":
            menu_analyze()
        elif choice == "4":
            menu_help()
        elif choice == "0":
            print("\n  再見！\n")
            break
        else:
            print("  [錯誤] 無效選項，請輸入 0-4")

        input("\n  按 Enter 返回主選單...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  再見！\n")
