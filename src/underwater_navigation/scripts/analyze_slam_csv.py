#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_CSV = Path(
    "/home/xiaoyang/water_robot/auv_ws/slam_logs/latest/slam_debug_latest.csv"
)
DEFAULT_OUTPUT_DIR = Path("/home/xiaoyang/water_robot/auv_ws/slam_logs/figures/debug")
DEFAULT_METRICS_SUMMARY = Path(
    "/home/xiaoyang/water_robot/auv_ws/slam_logs/metrics_summary.csv"
)

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
    "dvl_time_diff",
    "dvl_residual_x",
    "dvl_residual_y",
    "dvl_residual_z",
]

METRIC_FIELDS = [
    "experiment_name",
    "samples",
    "duration_s",
    "rmse_position",
    "mean_position_error",
    "max_position_error",
    "std_position_error",
    "final_position_error",
    "rmse_x",
    "rmse_y",
    "rmse_z",
    "mean_abs_x",
    "mean_abs_y",
    "mean_abs_z",
    "max_abs_x",
    "max_abs_y",
    "max_abs_z",
    "dvl_match_rate",
    "mean_dvl_time_diff",
    "max_dvl_time_diff",
    "rmse_dvl_residual_x",
    "rmse_dvl_residual_y",
    "rmse_dvl_residual_z",
    "sonar_up_match_rate",
    "sonar_down_match_rate",
    "sonar_left_match_rate",
    "sonar_right_match_rate",
    "rmse_sonar_up_residual",
    "rmse_sonar_down_residual",
    "rmse_sonar_left_residual",
    "rmse_sonar_right_residual",
    "mean_abs_sonar_up_residual",
    "mean_abs_sonar_down_residual",
    "mean_abs_sonar_left_residual",
    "mean_abs_sonar_right_residual",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze /slam_debug CSV logs and generate paper figures."
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--output_dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--experiment_name", default="imu_depth_dvl_6dof")
    parser.add_argument(
        "--paper_only",
        default="false",
        choices=("true", "false"),
        help="If true, generate only trajectory/error figures suitable for paper use.",
    )
    parser.add_argument(
        "--include_residuals",
        default="false",
        choices=("true", "false"),
        help="If true, also generate residual debug figures when paper_only is true.",
    )
    return parser.parse_args()


def require_fields(dataframe: pd.DataFrame) -> None:
    missing_fields = [field for field in REQUIRED_FIELDS if field not in dataframe.columns]
    if missing_fields:
        missing_text = ", ".join(missing_fields)
        raise SystemExit(f"CSV is missing required field(s): {missing_text}")


def numeric_series(dataframe: pd.DataFrame, field: str) -> pd.Series:
    return pd.to_numeric(dataframe[field], errors="coerce")


def load_csv(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise SystemExit(f"CSV file does not exist: {csv_path}")

    dataframe = pd.read_csv(csv_path)
    require_fields(dataframe)

    for field in REQUIRED_FIELDS:
        dataframe[field] = numeric_series(dataframe, field)

    for field in dataframe.columns:
        if field.startswith("sonar_"):
            dataframe[field] = numeric_series(dataframe, field)

    dataframe = dataframe.dropna(subset=REQUIRED_FIELDS).copy()
    if dataframe.empty:
        raise SystemExit("CSV contains no valid rows after numeric field conversion.")

    dataframe = dataframe.sort_values("time").reset_index(drop=True)
    return dataframe


def save_figure(fig, output_path: Path) -> None:
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    print(f"Saved figure: {output_path}")


def plot_trajectory_xy(dataframe: pd.DataFrame, output_dir: Path, experiment_name: str):
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    ax.plot(dataframe["gt_x"], dataframe["gt_y"], label="Ground Truth", linewidth=2.0)
    ax.plot(dataframe["slam_x"], dataframe["slam_y"], label="SLAM Estimate", linewidth=1.8)
    ax.set_title("Trajectory Comparison")
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.grid(True)
    ax.legend()
    ax.axis("equal")
    save_figure(fig, output_dir / f"trajectory_xy_{experiment_name}.png")


def plot_trajectory_xz(dataframe: pd.DataFrame, output_dir: Path, experiment_name: str):
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    ax.plot(dataframe["gt_x"], dataframe["gt_z"], label="Ground Truth", linewidth=2.0)
    ax.plot(dataframe["slam_x"], dataframe["slam_z"], label="SLAM Estimate", linewidth=1.8)
    ax.set_title("X-Z Trajectory Comparison")
    ax.set_xlabel("X / m")
    ax.set_ylabel("Z / m")
    ax.grid(True)
    ax.legend()
    ax.axis("equal")
    save_figure(fig, output_dir / f"trajectory_xz_{experiment_name}.png")


def plot_error_xyz(dataframe: pd.DataFrame, output_dir: Path, experiment_name: str):
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(dataframe["time"], dataframe["err_x"], label="Error X", linewidth=1.5)
    ax.plot(dataframe["time"], dataframe["err_y"], label="Error Y", linewidth=1.5)
    ax.plot(dataframe["time"], dataframe["err_z"], label="Error Z", linewidth=1.5)
    ax.set_title("Position Error in X/Y/Z")
    ax.set_xlabel("Time / s")
    ax.set_ylabel("Position Error / m")
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir / f"error_xyz_{experiment_name}.png")


def plot_position_error_norm(
    dataframe: pd.DataFrame, output_dir: Path, experiment_name: str
):
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(
        dataframe["time"],
        dataframe["position_error_norm"],
        label="Position Error Norm",
        linewidth=1.8,
    )
    ax.set_title("Position Error Norm")
    ax.set_xlabel("Time / s")
    ax.set_ylabel("Position Error Norm / m")
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir / f"position_error_norm_{experiment_name}.png")


def plot_dvl_residual(dataframe: pd.DataFrame, output_dir: Path, experiment_name: str):
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(
        dataframe["time"],
        dataframe["dvl_residual_x"],
        label="DVL Residual X",
        linewidth=1.5,
    )
    ax.plot(
        dataframe["time"],
        dataframe["dvl_residual_y"],
        label="DVL Residual Y",
        linewidth=1.5,
    )
    ax.plot(
        dataframe["time"],
        dataframe["dvl_residual_z"],
        label="DVL Residual Z",
        linewidth=1.5,
    )
    ax.set_title("DVL Residual")
    ax.set_xlabel("Time / s")
    ax.set_ylabel("DVL Residual / m/s")
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir / f"dvl_residual_{experiment_name}.png")


def rmse(values: pd.Series) -> float:
    array = values.to_numpy(dtype=float)
    return float(np.sqrt(np.mean(np.square(array))))


def compute_metrics(dataframe: pd.DataFrame, experiment_name: str) -> dict:
    dvl_matched = dataframe["dvl_matched"].astype(bool)
    position_error = dataframe["position_error_norm"]

    metrics = {
        "experiment_name": experiment_name,
        "samples": int(len(dataframe)),
        "duration_s": float(dataframe["time"].iloc[-1] - dataframe["time"].iloc[0]),
        "rmse_position": rmse(position_error),
        "mean_position_error": float(position_error.mean()),
        "max_position_error": float(position_error.max()),
        "std_position_error": float(position_error.std(ddof=0)),
        "final_position_error": float(position_error.iloc[-1]),
        "rmse_x": rmse(dataframe["err_x"]),
        "rmse_y": rmse(dataframe["err_y"]),
        "rmse_z": rmse(dataframe["err_z"]),
        "mean_abs_x": float(dataframe["err_x"].abs().mean()),
        "mean_abs_y": float(dataframe["err_y"].abs().mean()),
        "mean_abs_z": float(dataframe["err_z"].abs().mean()),
        "max_abs_x": float(dataframe["err_x"].abs().max()),
        "max_abs_y": float(dataframe["err_y"].abs().max()),
        "max_abs_z": float(dataframe["err_z"].abs().max()),
        "dvl_match_rate": float(dvl_matched.mean()),
        "mean_dvl_time_diff": float(dataframe["dvl_time_diff"].mean()),
        "max_dvl_time_diff": float(dataframe["dvl_time_diff"].max()),
        "rmse_dvl_residual_x": rmse(dataframe["dvl_residual_x"]),
        "rmse_dvl_residual_y": rmse(dataframe["dvl_residual_y"]),
        "rmse_dvl_residual_z": rmse(dataframe["dvl_residual_z"]),
    }

    for name in ("up", "down", "left", "right"):
        matched_field = f"sonar_{name}_matched"
        residual_field = f"sonar_{name}_residual"
        if matched_field in dataframe.columns:
            matched = dataframe[matched_field].fillna(0).astype(bool)
            metrics[f"sonar_{name}_match_rate"] = float(matched.mean())
        else:
            metrics[f"sonar_{name}_match_rate"] = ""

        if residual_field in dataframe.columns:
            residual = numeric_series(dataframe, residual_field).dropna()
            if not residual.empty:
                metrics[f"rmse_sonar_{name}_residual"] = rmse(residual)
                metrics[f"mean_abs_sonar_{name}_residual"] = float(residual.abs().mean())
            else:
                metrics[f"rmse_sonar_{name}_residual"] = ""
                metrics[f"mean_abs_sonar_{name}_residual"] = ""
        else:
            metrics[f"rmse_sonar_{name}_residual"] = ""
            metrics[f"mean_abs_sonar_{name}_residual"] = ""

    return metrics


def write_metrics_csv(path: Path, metrics: dict, append: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = path.exists()
    mode = "a" if append else "w"
    with path.open(mode, newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=METRIC_FIELDS)
        if not append or not file_exists:
            writer.writeheader()
        writer.writerow({field: metrics[field] for field in METRIC_FIELDS})


def print_metrics(metrics: dict) -> None:
    print("Metrics:")
    for field in METRIC_FIELDS:
        print(f"  {field}: {metrics[field]}")


def main() -> None:
    args = parse_args()
    dataframe = load_csv(args.csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    plot_trajectory_xy(dataframe, args.output_dir, args.experiment_name)
    plot_trajectory_xz(dataframe, args.output_dir, args.experiment_name)
    plot_error_xyz(dataframe, args.output_dir, args.experiment_name)
    plot_position_error_norm(dataframe, args.output_dir, args.experiment_name)
    paper_only = args.paper_only == "true"
    include_residuals = args.include_residuals == "true"
    if not paper_only or include_residuals:
        plot_dvl_residual(dataframe, args.output_dir, args.experiment_name)

    metrics = compute_metrics(dataframe, args.experiment_name)
    print_metrics(metrics)

    per_experiment_metrics_path = args.output_dir / f"metrics_{args.experiment_name}.csv"
    write_metrics_csv(per_experiment_metrics_path, metrics, append=False)
    write_metrics_csv(DEFAULT_METRICS_SUMMARY, metrics, append=True)
    print(f"Saved metrics: {per_experiment_metrics_path}")
    print(f"Appended metrics summary: {DEFAULT_METRICS_SUMMARY}")


if __name__ == "__main__":
    main()
