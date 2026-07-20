#!/usr/bin/env python3

import csv
import math
import re
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BASIC_FIELDS = [
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

OPTIONAL_FIELDS = [
    "yaw_error",
    "depth_residual",
    "mag_residual",
    "mag_matched",
    "mag_time_diff",
    "mag_yaw_measured",
    "sonar_up_matched",
    "sonar_down_matched",
    "sonar_left_matched",
    "sonar_right_matched",
    "sonar_up_residual",
    "sonar_down_residual",
    "sonar_left_residual",
    "sonar_right_residual",
    "sonar_up_range",
    "sonar_down_range",
    "sonar_left_range",
    "sonar_right_range",
]

SCALAR_PATTERN = re.compile(
    r"(?<!\S)([A-Za-z_][A-Za-z0-9_]*)\s+"
    r"(true|false|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
)


class SlamDebugRecorder(Node):
    def __init__(self) -> None:
        super().__init__("slam_debug_recorder")

        self.declare_parameter("slam_debug_topic", "/slam_debug")
        self.declare_parameter("experiment_name", "imu_depth_dvl_6dof")
        self.declare_parameter("log_root", "/home/xiaoyang/water_robot/auv_ws/slam_logs")

        self.topic = self.get_parameter("slam_debug_topic").value
        self.experiment_name = self.get_parameter("experiment_name").value
        self.log_root = Path(self.get_parameter("log_root").value)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_dir = self.log_root / "experiments" / self.experiment_name
        latest_dir = self.log_root / "latest"
        experiment_dir.mkdir(parents=True, exist_ok=True)
        latest_dir.mkdir(parents=True, exist_ok=True)

        self.experiment_csv_path = (
            experiment_dir
            / f"slam_debug_{self.experiment_name}_{timestamp}.csv"
        )
        self.latest_csv_path = latest_dir / "slam_debug_latest.csv"

        self.csv_fields = BASIC_FIELDS + OPTIONAL_FIELDS
        self.experiment_file = self.experiment_csv_path.open("w", newline="")
        self.latest_file = self.latest_csv_path.open("w", newline="")
        self.experiment_writer = csv.DictWriter(
            self.experiment_file,
            fieldnames=self.csv_fields,
        )
        self.latest_writer = csv.DictWriter(
            self.latest_file,
            fieldnames=self.csv_fields,
        )
        self.experiment_writer.writeheader()
        self.latest_writer.writeheader()
        self.experiment_file.flush()
        self.latest_file.flush()

        self.get_logger().info(
            f"Recording /slam_debug experiment CSV: {self.experiment_csv_path}"
        )
        self.get_logger().info(
            f"Recording /slam_debug latest CSV: {self.latest_csv_path}"
        )

        self.last_parse_warn_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            String,
            self.topic,
            self.slam_debug_callback,
            100,
        )

    def slam_debug_callback(self, msg: String) -> None:
        record = self.parse_slam_debug_text(msg.data)
        if record is None:
            now = self.get_clock().now()
            if (now - self.last_parse_warn_time).nanoseconds > 5_000_000_000:
                self.get_logger().warn(
                    "Drop /slam_debug message because required fields are missing"
                )
                self.last_parse_warn_time = now
            return

        self.experiment_writer.writerow(record)
        self.latest_writer.writerow(record)
        self.experiment_file.flush()
        self.latest_file.flush()

    def parse_slam_debug_text(self, text: str) -> Optional[Dict[str, object]]:
        values: Dict[str, object] = {}
        for key, raw_value in SCALAR_PATTERN.findall(text):
            if raw_value == "true":
                values[key] = 1
            elif raw_value == "false":
                values[key] = 0
            else:
                values[key] = float(raw_value)

        self.fill_vector_residual_fallback(text, values)

        if "position_error_norm" not in values:
            if all(key in values for key in ("err_x", "err_y", "err_z")):
                values["position_error_norm"] = math.sqrt(
                    values["err_x"] ** 2
                    + values["err_y"] ** 2
                    + values["err_z"] ** 2
                )

        if "dvl_matched" in values:
            values["dvl_matched"] = int(values["dvl_matched"])
        if "mag_matched" in values:
            values["mag_matched"] = int(values["mag_matched"])
        for field in (
            "sonar_up_matched",
            "sonar_down_matched",
            "sonar_left_matched",
            "sonar_right_matched",
        ):
            if field in values:
                values[field] = int(values[field])

        missing_fields = [field for field in BASIC_FIELDS if field not in values]
        if missing_fields:
            return None

        return {field: values.get(field, "") for field in self.csv_fields}

    @staticmethod
    def fill_vector_residual_fallback(text: str, values: Dict[str, object]) -> None:
        if all(
            field in values
            for field in ("dvl_residual_x", "dvl_residual_y", "dvl_residual_z")
        ):
            return

        match = re.search(
            r"dvl_residual\s+\[\s*"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*,\s*"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*,\s*"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*\]",
            text,
        )
        if not match:
            return

        values.setdefault("dvl_residual_x", float(match.group(1)))
        values.setdefault("dvl_residual_y", float(match.group(2)))
        values.setdefault("dvl_residual_z", float(match.group(3)))

    def close_files(self) -> None:
        for file_handle in (self.experiment_file, self.latest_file):
            if not file_handle.closed:
                file_handle.flush()
                file_handle.close()

    def destroy_node(self) -> bool:
        self.close_files()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamDebugRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
