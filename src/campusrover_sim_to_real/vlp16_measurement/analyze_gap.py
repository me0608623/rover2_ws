#!/usr/bin/env python3
"""
VLP-16 LiDAR 多組量測批次分析腳本

讀取 measurement_node.py 產生的多組 JSON 量測檔案，
進行偏差模式分析、噪聲特性分析，並輸出：
  - 3 張統計圖表（PNG）
  - Isaac Lab 噪聲補償參數（.py）
  - Markdown 分析報告（.md）

使用方式：
    python3 analyze_gap.py --data_dir ./data --output_dir ./results --material white_wall
"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stats_utils import (
    BIAS_TYPE_DISTANCE_DEPENDENT,
    BIAS_TYPE_FIXED,
    compute_statistics,
    format_statistics_report,
    linear_regression,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ---------------------------------------------------------------------------
# 輔助函式
# ---------------------------------------------------------------------------

# 字體設定快取，避免重複掃描檔案系統
_font_setup_result = None


def setup_matplotlib_fonts():
    """嘗試設定支援中文的字體，失敗則降級為英文（結果快取）。"""
    global _font_setup_result
    if _font_setup_result is not None:
        return _font_setup_result

    if not HAS_MATPLOTLIB:
        _font_setup_result = False
        return False

    cjk_fonts = [
        "Noto Sans CJK TC",
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "AR PL UMing TW",
        "DejaVu Sans",
    ]

    # 使用 singleton 而非建構新 FontManager
    from matplotlib.font_manager import fontManager
    available = {f.name for f in fontManager.ttflist}

    for font in cjk_fonts:
        if font in available:
            plt.rcParams["font.sans-serif"] = [font, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            _font_setup_result = font != "DejaVu Sans"
            return _font_setup_result

    plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    _font_setup_result = False
    return False


def _save_figure(fig, output_dir: str, filename: str):
    """儲存 matplotlib 圖表並關閉。"""
    fig.tight_layout()
    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[圖表] 已儲存：{path}")


def _label(cjk: str, en: str, has_cjk: bool) -> str:
    """依 CJK 字體可用性選擇標籤文字。"""
    return cjk if has_cjk else en


def parse_args():
    """解析命令列參數。"""
    parser = argparse.ArgumentParser(
        description="VLP-16 LiDAR 測距誤差批次分析",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "範例：\n"
            "  python3 analyze_gap.py --data_dir ./data --output_dir ./results\n"
            "  python3 analyze_gap.py --data_dir ./data --material white_wall\n"
        ),
    )
    parser.add_argument("--data_dir", type=str, default="./data", help="JSON 資料目錄（預設：./data）")
    parser.add_argument("--output_dir", type=str, default="./results", help="結果輸出目錄（預設：./results）")
    parser.add_argument("--material", type=str, default="all", help="篩選材質（預設：all，不篩選）")
    parser.add_argument("--min_samples", type=int, default=1, help="每個距離至少需要的有效量測數（預設：1）")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# 資料載入
# ---------------------------------------------------------------------------

def load_measurements(data_dir: str, material: str) -> list:
    """
    載入指定目錄下的所有量測 JSON 檔案。

    Parameters
    ----------
    data_dir : str
        JSON 資料目錄路徑
    material : str
        篩選材質，"all" 表示不篩選

    Returns
    -------
    list[dict]
        量測資料字典列表
    """
    pattern = os.path.join(data_dir, "measurement_*.json")
    files = sorted(glob.glob(pattern))

    if not files:
        print(f"[錯誤] 在 '{os.path.abspath(data_dir)}' 找不到任何 measurement_*.json 檔案")
        sys.exit(1)

    measurements = []
    skipped = 0

    for filepath in files:
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            if "metadata" not in data or "raw_samples" not in data:
                print(f"[警告] 跳過格式不正確的檔案：{os.path.basename(filepath)}")
                skipped += 1
                continue

            if "ground_truth_m" not in data["metadata"]:
                print(f"[警告] 跳過缺少 ground_truth_m 的檔案：{os.path.basename(filepath)}")
                skipped += 1
                continue

            file_material = data["metadata"].get("material", "unknown")
            if material != "all" and file_material != material:
                continue

            measurements.append(data)

        except json.JSONDecodeError as e:
            print(f"[警告] JSON 解析錯誤，跳過 {os.path.basename(filepath)}: {e}")
            skipped += 1
        except OSError as e:
            print(f"[警告] 無法讀取 {os.path.basename(filepath)}: {e}")
            skipped += 1

    if not measurements:
        material_hint = f"（材質篩選：{material}）" if material != "all" else ""
        print(f"[錯誤] 沒有符合條件的量測資料{material_hint}")
        sys.exit(1)

    print(f"\n[載入] 共讀取 {len(measurements)} 個量測檔案" +
          (f"（跳過 {skipped} 個）" if skipped else ""))

    return measurements


def group_by_distance(measurements: list) -> dict:
    """
    依 Ground Truth 距離分組，合併 raw_samples。

    Returns
    -------
    dict[float, list[float]]
        {GT 距離: [所有合併的 raw_samples]}
    """
    groups = {}
    for m in measurements:
        gt = m["metadata"]["ground_truth_m"]
        # 四捨五入到 mm 避免浮點精度問題
        gt_key = round(gt, 3)
        if gt_key not in groups:
            groups[gt_key] = []
        groups[gt_key].extend(m["raw_samples"])

    sorted_distances = sorted(groups.keys())
    print(f"[分組] {len(sorted_distances)} 個距離點: {[f'{d:.1f}m' for d in sorted_distances]}")
    for d in sorted_distances:
        print(f"       {d:.3f}m → {len(groups[d])} 筆樣本")

    return groups


# ---------------------------------------------------------------------------
# 統計分析
# ---------------------------------------------------------------------------

def analyze_groups(groups: dict, r2_threshold: float = 0.3) -> dict:
    """
    對每個距離組別計算統計，並進行偏差模式判斷。

    Returns
    -------
    dict
        包含 per_distance 統計、bias_model、noise_model、overall 彙總、all_errors
    """
    sorted_distances = sorted(groups.keys())
    per_distance = {}
    gt_list = []
    bias_list = []
    std_list = []
    all_errors = []

    for gt in sorted_distances:
        samples = groups[gt]
        stats = compute_statistics(samples, gt)
        stats["n_valid_frames"] = len(samples)
        per_distance[gt] = stats

        gt_list.append(gt)
        bias_list.append(stats["bias_m"])
        std_list.append(stats["std_m"])

        errors = np.array(samples) - gt
        all_errors.extend(errors.tolist())

    bias_reg = linear_regression(gt_list, bias_list)
    bias_type = BIAS_TYPE_DISTANCE_DEPENDENT if bias_reg["r_squared"] >= r2_threshold else BIAS_TYPE_FIXED
    bias_reg["type"] = bias_type

    noise_reg = linear_regression(gt_list, std_list)

    all_biases = np.array(bias_list)
    all_stds = np.array(std_list)
    all_errors_arr = np.array(all_errors)

    within_3cm_values = [per_distance[d]["within_3cm_pct"] for d in sorted_distances]
    avg_within_3cm = float(np.mean(within_3cm_values))
    dropout_rate = max(0.0, (100.0 - avg_within_3cm) / 100.0)

    overall = {
        "bias_mean": float(np.mean(all_biases)),
        "bias_min": float(np.min(all_biases)),
        "bias_max": float(np.max(all_biases)),
        "std_mean": float(np.mean(all_stds)),
        "std_min": float(np.min(all_stds)),
        "std_max": float(np.max(all_stds)),
        "all_errors_mean": float(np.mean(all_errors_arr)),
        "all_errors_std": float(np.std(all_errors_arr)),
        "dropout_rate": dropout_rate,
        "avg_within_3cm_pct": avg_within_3cm,
    }

    print("\n" + "=" * 60)
    print("  偏差模式分析結果")
    print("=" * 60)
    print(f"  Bias 線性回歸：slope={bias_reg['slope']:.6f}, "
          f"intercept={bias_reg['intercept']:.6f}, R²={bias_reg['r_squared']:.4f}")
    if bias_type == BIAS_TYPE_FIXED:
        print(f"  → 判定：固定偏差（R²={bias_reg['r_squared']:.4f} < {r2_threshold}）")
        print(f"    建議補償值：{overall['bias_mean']:.4f} m（所有距離的 bias 均值）")
    else:
        print(f"  → 判定：距離相關偏差（R²={bias_reg['r_squared']:.4f} ≥ {r2_threshold}）")
        print(f"    補償公式：bias = {bias_reg['slope']:.6f} × d + {bias_reg['intercept']:.6f}")

    print(f"\n  Noise 線性回歸：slope={noise_reg['slope']:.6f}, "
          f"intercept={noise_reg['intercept']:.6f}, R²={noise_reg['r_squared']:.4f}")
    if noise_reg["r_squared"] >= r2_threshold:
        print(f"  → 噪聲隨距離增大（R²={noise_reg['r_squared']:.4f} ≥ {r2_threshold}）")
    else:
        print(f"  → 噪聲與距離無顯著相關（R²={noise_reg['r_squared']:.4f} < {r2_threshold}）")
    print("=" * 60)

    return {
        "per_distance": per_distance,
        "distances": sorted_distances,
        "bias_model": bias_reg,
        "noise_model": noise_reg,
        "overall": overall,
        "all_errors": all_errors_arr,
        "r2_threshold": r2_threshold,
    }


# ---------------------------------------------------------------------------
# 繪圖
# ---------------------------------------------------------------------------

def generate_plots(analysis: dict, output_dir: str):
    """產生 bias 與 noise 分析圖表。"""
    if not HAS_MATPLOTLIB:
        print("[警告] 未安裝 matplotlib，跳過圖表產生。請執行：pip3 install matplotlib")
        return

    has_cjk = setup_matplotlib_fonts()

    distances = analysis["distances"]
    per_dist = analysis["per_distance"]
    bias_model = analysis["bias_model"]
    noise_model = analysis["noise_model"]

    biases = [per_dist[d]["bias_m"] for d in distances]
    stds = [per_dist[d]["std_m"] for d in distances]

    # 圖一：Bias Analysis
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.errorbar(distances, biases, yerr=stds, fmt="o", capsize=5,
                color="steelblue", ecolor="gray", label="Bias ± Std")

    if len(distances) >= 2:
        fit_x = np.linspace(min(distances) * 0.8, max(distances) * 1.2, 100)
        fit_y = bias_model["slope"] * fit_x + bias_model["intercept"]
        ax.plot(fit_x, fit_y, "--", color="darkorange", linewidth=1.5,
                label=f"Linear fit (R²={bias_model['r_squared']:.3f})")

    ax.axhline(y=0.03, color="red", linestyle=":", linewidth=1, label="±3cm spec")
    ax.axhline(y=-0.03, color="red", linestyle=":", linewidth=1)
    ax.axhline(y=0, color="gray", linestyle="-", linewidth=0.5, alpha=0.5)

    ax.set_title(_label("偏差分析 (Bias Analysis)", "Bias Analysis", has_cjk), fontsize=14)
    ax.set_xlabel(_label("Ground Truth 距離 (m)", "Ground Truth Distance (m)", has_cjk), fontsize=12)
    ax.set_ylabel(_label("偏差 Bias (m)", "Bias (m)", has_cjk), fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    bias_type_text = ("Fixed Bias" if bias_model["type"] == BIAS_TYPE_FIXED
                      else "Distance-Dependent Bias")
    ax.text(0.02, 0.98, f"R² = {bias_model['r_squared']:.4f}\nType: {bias_type_text}",
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    _save_figure(fig, output_dir, "bias_analysis.png")

    # 圖二：Noise Analysis
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(distances, stds, "o-", color="steelblue", linewidth=1.5,
            markersize=8, label="Std Dev")

    if len(distances) >= 2:
        fit_x = np.linspace(min(distances) * 0.8, max(distances) * 1.2, 100)
        fit_y = noise_model["slope"] * fit_x + noise_model["intercept"]
        ax.plot(fit_x, fit_y, "--", color="darkorange", linewidth=1.5,
                label=f"Linear fit (R²={noise_model['r_squared']:.3f})")

    ax.set_title(_label("雜訊分析 (Noise Analysis)", "Noise Analysis", has_cjk), fontsize=14)
    ax.set_xlabel(_label("Ground Truth 距離 (m)", "Ground Truth Distance (m)", has_cjk), fontsize=12)
    ax.set_ylabel(_label("標準差 Std (m)", "Std Dev (m)", has_cjk), fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    ax.text(0.02, 0.98, f"R² = {noise_model['r_squared']:.4f}",
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    _save_figure(fig, output_dir, "noise_analysis.png")


def generate_error_distribution_plot(analysis: dict, output_dir: str):
    """產生誤差分佈圖（直接從 analysis dict 取已計算的 all_errors）。"""
    if not HAS_MATPLOTLIB:
        return

    has_cjk = setup_matplotlib_fonts()
    all_errors = analysis.get("all_errors")

    if all_errors is None or len(all_errors) == 0:
        return

    fig, ax = plt.subplots(figsize=(10, 6))

    n_bins = min(50, max(10, len(all_errors) // 5))
    ax.hist(all_errors, bins=n_bins, density=True, alpha=0.7,
            color="steelblue", edgecolor="white", label="Error Distribution")

    err_mean = float(np.mean(all_errors))
    err_std = float(np.std(all_errors))
    if err_std > 0:
        x_gauss = np.linspace(all_errors.min(), all_errors.max(), 200)
        y_gauss = (1.0 / (err_std * np.sqrt(2 * np.pi))) * \
                  np.exp(-0.5 * ((x_gauss - err_mean) / err_std) ** 2)
        ax.plot(x_gauss, y_gauss, "r-", linewidth=2,
                label=f"Gaussian (μ={err_mean:.4f}, σ={err_std:.4f})")

    ax.axvline(x=0.03, color="red", linestyle=":", linewidth=1, alpha=0.7, label="±3cm")
    ax.axvline(x=-0.03, color="red", linestyle=":", linewidth=1, alpha=0.7)
    ax.axvline(x=0, color="gray", linestyle="-", linewidth=0.5, alpha=0.5)

    ax.set_title(_label("誤差分佈 (Error Distribution)", "Error Distribution", has_cjk), fontsize=14)
    ax.set_xlabel(_label("誤差 (m) [量測值 - GT]", "Error (m) [Measured - GT]", has_cjk), fontsize=12)
    ax.set_ylabel(_label("機率密度", "Probability Density", has_cjk), fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    ax.text(0.02, 0.98,
            f"N = {len(all_errors)}\nμ = {err_mean:.4f} m\nσ = {err_std:.4f} m",
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    _save_figure(fig, output_dir, "error_distribution.png")


# ---------------------------------------------------------------------------
# Isaac Lab 參數輸出
# ---------------------------------------------------------------------------

def generate_noise_params(analysis: dict, n_files: int, material: str, output_dir: str):
    """產生可直接 import 的 Isaac Lab 噪聲補償參數檔案。"""
    overall = analysis["overall"]
    bias_model = analysis["bias_model"]
    timestamp = datetime.now().isoformat()

    bias_mean = overall["bias_mean"]
    bias_range_min = bias_mean * 0.8 if bias_mean >= 0 else bias_mean * 1.2
    bias_range_max = bias_mean * 1.2 if bias_mean >= 0 else bias_mean * 0.8

    std_mean = overall["std_mean"]
    std_range_min = std_mean * 0.5
    std_range_max = std_mean * 1.5

    dropout_rate = overall["dropout_rate"]
    dropout_min = max(0.0, dropout_rate * 0.5)
    dropout_max = min(1.0, dropout_rate * 1.5)

    content = f'''# 自動生成的 VLP-16 噪聲補償參數
# 生成時間：{timestamp}
# 資料來源：{n_files} 個量測檔案，材質：{material}
# 偏差類型：{bias_model["type"]}

# Bias 補償（系統性偏差）
LIDAR_BIAS_MEAN   = {bias_mean:.6f}   # 單位：m，所有距離的平均 bias
LIDAR_BIAS_RANGE  = ({bias_range_min:.6f}, {bias_range_max:.6f})  # DR 範圍：±20%

# 高斯噪聲（隨機誤差）
LIDAR_NOISE_STD_MEAN  = {std_mean:.6f}   # 單位：m，所有距離的平均 std
LIDAR_NOISE_STD_RANGE = ({std_range_min:.6f}, {std_range_max:.6f})  # DR 範圍：×0.5 到 ×1.5

# Dropout 率（依 within_3cm_pct 反推）
LIDAR_DROPOUT_RATE_MEAN  = {dropout_rate:.6f}  # 估計丟點率
LIDAR_DROPOUT_RATE_RANGE = ({dropout_min:.6f}, {dropout_max:.6f})  # DR 範圍

# 偏差類型旗標（若為 distance_dependent，需用線性公式補償）
LIDAR_BIAS_TYPE      = "{bias_model["type"]}"
LIDAR_BIAS_SLOPE     = {bias_model["slope"]:.6f}  # 僅 distance_dependent 時有效
LIDAR_BIAS_INTERCEPT = {bias_model["intercept"]:.6f}
'''

    filepath = os.path.join(output_dir, "isaac_lab_noise_params.py")
    try:
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"\n[參數] Isaac Lab 噪聲參數已儲存至：{filepath}")
    except OSError as e:
        print(f"[錯誤] 無法儲存參數檔案：{e}")


# ---------------------------------------------------------------------------
# Markdown 報告
# ---------------------------------------------------------------------------

def generate_report(analysis: dict, measurements: list, material: str, output_dir: str):
    """產生 Markdown 格式的分析報告。"""
    distances = analysis["distances"]
    per_dist = analysis["per_distance"]
    bias_model = analysis["bias_model"]
    noise_model = analysis["noise_model"]
    overall = analysis["overall"]
    r2_threshold = analysis["r2_threshold"]
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    n_files = len(measurements)

    lines = [
        "# VLP-16 LiDAR 測距偏差分析報告",
        "",
        "## 實驗摘要",
        "",
        f"- **分析日期**：{timestamp}",
        f"- **資料筆數**：{n_files} 個量測檔案",
        f"- **材質**：{material}",
        f"- **距離範圍**：{min(distances):.1f}m ~ {max(distances):.1f}m（{len(distances)} 個距離點）",
        "",
        "## 各距離統計數據",
        "",
        "| GT距離 (m) | Bias (m) | MAE (m) | RMSE (m) | Std (m) | ±3cm (%) | ±5cm (%) | 樣本數 |",
        "|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|",
    ]

    for d in distances:
        s = per_dist[d]
        lines.append(
            f"| {d:.2f} | {s['bias_m']:+.4f} | {s['mae_m']:.4f} | "
            f"{s['rmse_m']:.4f} | {s['std_m']:.4f} | "
            f"{s['within_3cm_pct']:.1f} | {s['within_5cm_pct']:.1f} | "
            f"{s['n_valid_frames']} |"
        )

    lines.extend([
        "",
        "## 偏差模式判斷",
        "",
        "### Bias 線性回歸",
        "",
        f"- **斜率 (slope)**：{bias_model['slope']:.6f}",
        f"- **截距 (intercept)**：{bias_model['intercept']:.6f}",
        f"- **R²**：{bias_model['r_squared']:.4f}",
        "",
    ])

    if bias_model["type"] == BIAS_TYPE_FIXED:
        lines.extend([
            f"**結論：固定偏差**（R² = {bias_model['r_squared']:.4f} < {r2_threshold}）",
            "",
            "偏差與距離無顯著線性關係，屬於固定系統性偏差。",
            "此類偏差通常來自 TOF（Time-of-Flight）計時電路的固定延遲，",
            "可用所有距離的 bias 均值進行常數補償。",
            "",
            f"**建議補償值**：{overall['bias_mean']:.4f} m",
        ])
    else:
        lines.extend([
            f"**結論：距離相關偏差**（R² = {bias_model['r_squared']:.4f} ≥ {r2_threshold}）",
            "",
            "偏差隨距離呈線性變化，需使用線性公式進行補償：",
            "",
            "```",
            f"bias = {bias_model['slope']:.6f} × distance + {bias_model['intercept']:.6f}",
            "```",
            "",
            "此類偏差可能來自 TOF 計時的距離相關誤差或光學系統的幾何偏移。",
        ])

    lines.extend([
        "",
        "### Noise 線性回歸",
        "",
        f"- **斜率**：{noise_model['slope']:.6f}",
        f"- **截距**：{noise_model['intercept']:.6f}",
        f"- **R²**：{noise_model['r_squared']:.4f}",
        "",
    ])

    if noise_model["r_squared"] >= r2_threshold:
        lines.append("**結論**：噪聲隨距離增大，遠距離量測不確定度較高。")
    else:
        lines.append("**結論**：噪聲與距離無顯著相關，各距離噪聲水平相近。")

    lines.extend([
        "",
        "## Isaac Lab 補償參數",
        "",
        "以下參數已自動輸出至 `isaac_lab_noise_params.py`，可直接 import 使用：",
        "",
        "```python",
        f"LIDAR_BIAS_MEAN       = {overall['bias_mean']:.6f}",
        f"LIDAR_NOISE_STD_MEAN  = {overall['std_mean']:.6f}",
        f"LIDAR_DROPOUT_RATE    = {overall['dropout_rate']:.6f}",
        f"LIDAR_BIAS_TYPE       = \"{bias_model['type']}\"",
        f"LIDAR_BIAS_SLOPE      = {bias_model['slope']:.6f}",
        f"LIDAR_BIAS_INTERCEPT  = {bias_model['intercept']:.6f}",
        "```",
        "",
        "## 高斯假設檢驗",
        "",
        "根據中央極限定理（CLT），每幀取多個 LiDAR 回波的中位數後，",
        "幀間的距離量測值理論上應趨近常態分佈。",
        "",
        "請參考 `error_distribution.png` 的直方圖與高斯曲線疊合結果：",
        "",
        "- 若直方圖與高斯曲線吻合良好 → 支持高斯假設，可在 Isaac Lab 中使用高斯噪聲模型",
        "- 若存在明顯偏態或厚尾 → 需考慮混合分佈或 t 分佈模型",
        "",
        "**結論**：（請根據圖表目視結果填寫）",
        "",
        "---",
        f"*報告自動生成於 {timestamp}*",
    ])

    filepath = os.path.join(output_dir, "gap_analysis_report.md")
    try:
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        print(f"[報告] 分析報告已儲存至：{filepath}")
    except OSError as e:
        print(f"[錯誤] 無法儲存報告：{e}")


# ---------------------------------------------------------------------------
# 主程式
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    try:
        os.makedirs(args.output_dir, exist_ok=True)
    except OSError as e:
        print(f"[錯誤] 無法建立輸出目錄 {args.output_dir}: {e}")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("  VLP-16 LiDAR 測距偏差批次分析")
    print("=" * 60)

    measurements = load_measurements(args.data_dir, args.material)
    groups = group_by_distance(measurements)

    for gt, samples in list(groups.items()):
        if len(samples) < args.min_samples:
            print(f"[警告] 距離 {gt:.3f}m 僅有 {len(samples)} 筆樣本（低於 min_samples={args.min_samples}），已排除")
            del groups[gt]

    if not groups:
        print("[錯誤] 篩選後無有效距離組別")
        sys.exit(1)

    analysis = analyze_groups(groups)

    print("\n" + "-" * 60)
    print("  各距離詳細統計")
    print("-" * 60)
    for d in analysis["distances"]:
        stats = analysis["per_distance"][d]
        report = format_statistics_report(stats, d, stats["n_valid_frames"])
        print(report)

    generate_plots(analysis, args.output_dir)
    generate_error_distribution_plot(analysis, args.output_dir)

    generate_noise_params(analysis, len(measurements), args.material, args.output_dir)
    generate_report(analysis, measurements, args.material, args.output_dir)

    print("\n" + "=" * 60)
    print("  分析完成！輸出檔案：")
    print("=" * 60)
    print("  bias_analysis.png")
    print("  noise_analysis.png")
    print("  error_distribution.png")
    print("  isaac_lab_noise_params.py")
    print("  gap_analysis_report.md")
    print(f"\n  所有檔案位於：{os.path.abspath(args.output_dir)}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
