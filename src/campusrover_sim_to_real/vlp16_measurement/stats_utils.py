#!/usr/bin/env python3
"""
VLP-16 LiDAR 測距統計工具模組

提供共用的統計計算與格式化輸出函式，供 measurement_node.py 與 analyze_gap.py 共同使用。
僅依賴 numpy，確保兩支程式都能直接匯入。
"""

import numpy as np

# 偏差類型常數（避免散佈的字串字面量）
BIAS_TYPE_FIXED = "fixed"
BIAS_TYPE_DISTANCE_DEPENDENT = "distance_dependent"


def compute_statistics(samples: list, ground_truth: float) -> dict:
    """
    計算測距樣本的完整統計指標。

    Parameters
    ----------
    samples : list[float]
        各幀的中位數距離量測值（單位：m）
    ground_truth : float
        雷射測距儀量得的真實距離（單位：m）

    Returns
    -------
    dict
        包含 10 項統計指標，所有數值皆為 Python float（可直接 JSON 序列化）
    """
    arr = np.array(samples, dtype=np.float64)
    errors = arr - ground_truth
    abs_errors = np.abs(errors)

    mean_val = float(np.mean(arr))
    median_val = float(np.median(arr))
    std_val = float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0
    bias_val = float(np.mean(errors))
    mae_val = float(np.mean(abs_errors))
    rmse_val = float(np.sqrt(np.mean(errors ** 2)))
    p95_error_val = float(np.percentile(abs_errors, 95))
    within_3cm_val = float(np.sum(abs_errors < 0.03) / len(abs_errors) * 100.0)
    within_5cm_val = float(np.sum(abs_errors < 0.05) / len(abs_errors) * 100.0)

    return {
        "mean_m": mean_val,
        "median_m": median_val,
        "std_m": std_val,
        "bias_m": bias_val,
        "mae_m": mae_val,
        "rmse_m": rmse_val,
        "p95_error_m": p95_error_val,
        "within_3cm_pct": within_3cm_val,
        "within_5cm_pct": within_5cm_val,
    }


def format_statistics_report(stats: dict, ground_truth: float, n_valid_frames: int) -> str:
    """
    將統計結果格式化為繁體中文終端輸出字串。

    Parameters
    ----------
    stats : dict
        由 compute_statistics() 回傳的統計字典
    ground_truth : float
        Ground Truth 距離（m）
    n_valid_frames : int
        實際有效幀數

    Returns
    -------
    str
        多行格式化字串
    """
    lines = [
        "=" * 60,
        "  VLP-16 測距統計結果",
        "=" * 60,
        f"  Ground Truth 距離 : {ground_truth:.3f} m",
        f"  有效幀數          : {n_valid_frames}",
        "-" * 60,
        f"  均值 (mean)       : {stats['mean_m']:.4f} m",
        f"  中位數 (median)   : {stats['median_m']:.4f} m",
        f"  標準差 (std)      : {stats['std_m']:.4f} m",
        f"  偏差 (bias)       : {stats['bias_m']:+.4f} m  "
        f"({'高估' if stats['bias_m'] > 0 else '低估'})",
        f"  MAE              : {stats['mae_m']:.4f} m",
        f"  RMSE             : {stats['rmse_m']:.4f} m",
        f"  P95 誤差          : {stats['p95_error_m']:.4f} m",
        f"  ±3cm 內比例       : {stats['within_3cm_pct']:.1f}%",
        f"  ±5cm 內比例       : {stats['within_5cm_pct']:.1f}%",
        "=" * 60,
    ]
    return "\n".join(lines)


def linear_regression(x: list, y: list) -> dict:
    """
    使用 numpy 進行簡單線性回歸（避免 scipy 依賴）。

    Parameters
    ----------
    x : list[float]
        自變數
    y : list[float]
        因變數

    Returns
    -------
    dict
        包含 slope, intercept, r_squared
    """
    x_arr = np.array(x, dtype=np.float64)
    y_arr = np.array(y, dtype=np.float64)

    if len(x_arr) < 2:
        return {"slope": 0.0, "intercept": float(np.mean(y_arr)) if len(y_arr) > 0 else 0.0, "r_squared": 0.0}

    coeffs = np.polyfit(x_arr, y_arr, 1)
    slope = float(coeffs[0])
    intercept = float(coeffs[1])

    y_pred = slope * x_arr + intercept
    ss_res = float(np.sum((y_arr - y_pred) ** 2))
    ss_tot = float(np.sum((y_arr - np.mean(y_arr)) ** 2))
    r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    return {"slope": slope, "intercept": intercept, "r_squared": float(r_squared)}
