#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_OUTPUT_DIR = Path("/home/xiaoyang/water_robot/auv_ws/slam_logs/figures/paper")

REQUIRED_FIELDS = [
    "time",
    "gt_x",
    "gt_y",
    "gt_z",
    "slam_x",
    "slam_y",
    "slam_z",
    "err_x",
    "err_y",
    "err_z",
    "position_error_norm",
    "dvl_matched",
]

METRIC_FIELDS = [
    "label",
    "samples",
    "duration_s",
    "rmse_position",
    "mean_position_error",
    "max_position_error",
    "final_position_error",
    "rmse_x",
    "rmse_y",
    "rmse_z",
    "dvl_match_rate",
    "mag_match_rate",
    "sonar_up_match_rate",
    "sonar_down_match_rate",
    "sonar_left_match_rate",
    "sonar_right_match_rate",
    "rmse_sonar_up_residual",
    "rmse_sonar_down_residual",
    "rmse_sonar_left_residual",
    "rmse_sonar_right_residual",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate Baseline vs Baseline + Sonar comparison figures."
    )
    parser.add_argument("--baseline_csv", type=Path, required=True)
    parser.add_argument("--sonar_csv", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def numeric_series(dataframe: pd.DataFrame, field: str) -> pd.Series:
    return pd.to_numeric(dataframe[field], errors="coerce")


def load_csv(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise SystemExit(f"CSV file does not exist: {csv_path}")

    dataframe = pd.read_csv(csv_path)
    missing_fields = [field for field in REQUIRED_FIELDS if field not in dataframe.columns]
    if missing_fields:
        raise SystemExit(
            f"{csv_path} is missing required field(s): {', '.join(missing_fields)}"
        )

    for field in dataframe.columns:
        if field in REQUIRED_FIELDS or field.startswith(("mag_", "sonar_")):
            dataframe[field] = numeric_series(dataframe, field)

    dataframe = dataframe.dropna(subset=REQUIRED_FIELDS).copy()
    if dataframe.empty:
        raise SystemExit(f"{csv_path} contains no valid numeric data rows.")

    dataframe = dataframe.sort_values("time").reset_index(drop=True)
    dataframe["time_rel"] = dataframe["time"] - dataframe["time"].iloc[0]
    return dataframe


def save_figure(fig, output_path: Path) -> None:
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    print(f"Saved figure: {output_path}")


def plot_trajectory_xy(
    baseline: pd.DataFrame,
    sonar: pd.DataFrame,
    output_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    ax.plot(
        baseline["gt_x"],
        baseline["gt_y"],
        label="Baseline Ground Truth",
        linewidth=2.0,
    )
    ax.plot(
        baseline["slam_x"],
        baseline["slam_y"],
        label="Baseline Estimate",
        linewidth=1.7,
    )
    ax.plot(
        sonar["slam_x"],
        sonar["slam_y"],
        label="Baseline + Sonar Estimate",
        linewidth=1.7,
    )
    ax.set_title("Trajectory Comparison")
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.grid(True)
    ax.legend()
    ax.axis("equal")
    save_figure(fig, output_dir / "trajectory_xy_baseline_vs_sonar.png")


def plot_position_error_norm(
    baseline: pd.DataFrame,
    sonar: pd.DataFrame,
    output_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(
        baseline["time_rel"],
        baseline["position_error_norm"],
        label="Baseline",
        linewidth=1.7,
    )
    ax.plot(
        sonar["time_rel"],
        sonar["position_error_norm"],
        label="Baseline + Sonar",
        linewidth=1.7,
    )
    ax.set_title("Position Error Norm")
    ax.set_xlabel("Time / s")
    ax.set_ylabel("Position Error Norm / m")
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir / "position_error_norm_baseline_vs_sonar.png")


def plot_error_y(
    baseline: pd.DataFrame,
    sonar: pd.DataFrame,
    output_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(
        baseline["time_rel"],
        baseline["err_y"],
        label="Baseline",
        linewidth=1.7,
    )
    ax.plot(
        sonar["time_rel"],
        sonar["err_y"],
        label="Baseline + Sonar",
        linewidth=1.7,
    )
    ax.set_title("Y-Axis Position Error")
    ax.set_xlabel("Time / s")
    ax.set_ylabel("Y Error / m")
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir / "error_y_baseline_vs_sonar.png")


def rmse(values: pd.Series) -> float:
    array = values.to_numpy(dtype=float)
    return float(np.sqrt(np.mean(np.square(array))))


def match_rate(dataframe: pd.DataFrame, field: str):
    if field not in dataframe.columns:
        return ""
    return float(dataframe[field].fillna(0).astype(bool).mean())


def sonar_residual_rmse(dataframe: pd.DataFrame, name: str):
    matched_field = f"sonar_{name}_matched"
    residual_field = f"sonar_{name}_residual"
    if matched_field not in dataframe.columns or residual_field not in dataframe.columns:
        return ""

    matched = dataframe[matched_field].fillna(0).astype(bool)
    residual = dataframe.loc[matched, residual_field].dropna()
    if residual.empty:
        return ""
    return rmse(residual)


def compute_metrics(dataframe: pd.DataFrame, label: str) -> dict:
    metrics = {
        "label": label,
        "samples": int(len(dataframe)),
        "duration_s": float(dataframe["time"].iloc[-1] - dataframe["time"].iloc[0]),
        "rmse_position": rmse(dataframe["position_error_norm"]),
        "mean_position_error": float(dataframe["position_error_norm"].mean()),
        "max_position_error": float(dataframe["position_error_norm"].max()),
        "final_position_error": float(dataframe["position_error_norm"].iloc[-1]),
        "rmse_x": rmse(dataframe["err_x"]),
        "rmse_y": rmse(dataframe["err_y"]),
        "rmse_z": rmse(dataframe["err_z"]),
        "dvl_match_rate": match_rate(dataframe, "dvl_matched"),
        "mag_match_rate": match_rate(dataframe, "mag_matched"),
    }

    for name in ("up", "down", "left", "right"):
        metrics[f"sonar_{name}_match_rate"] = match_rate(
            dataframe,
            f"sonar_{name}_matched",
        )
        metrics[f"rmse_sonar_{name}_residual"] = sonar_residual_rmse(
            dataframe,
            name,
        )

    return metrics


def write_metrics(output_dir: Path, rows: list[dict]) -> Path:
    output_path = output_dir / "metrics_baseline_vs_sonar.csv"
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in METRIC_FIELDS})
    print(f"Saved metrics: {output_path}")
    return output_path


def main() -> None:
    args = parse_args()
    baseline = load_csv(args.baseline_csv)
    sonar = load_csv(args.sonar_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    plot_trajectory_xy(baseline, sonar, args.output_dir)
    plot_position_error_norm(baseline, sonar, args.output_dir)
    plot_error_y(baseline, sonar, args.output_dir)

    metrics_rows = [
        compute_metrics(baseline, "Baseline"),
        compute_metrics(sonar, "Baseline + Sonar"),
    ]
    write_metrics(args.output_dir, metrics_rows)


if __name__ == "__main__":
    main()
