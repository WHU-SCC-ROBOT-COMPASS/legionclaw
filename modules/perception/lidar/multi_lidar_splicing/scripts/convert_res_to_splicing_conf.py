#!/usr/bin/env python3
"""Convert calibration results in res.json into multi_lidar_splicing config format.

Input assumption:
- /livox/lidar: top lidar -> vehicle rear axle center
- other topics: source lidar -> top lidar

Output:
- bin/conf/config.json
- bin/conf/sensors/lidar_{front,mid,left,right,back}.json

All exported extrinsics are source lidar -> vehicle rear axle center.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

TOP_LIDAR_TOPIC = "/livox/lidar"

# Keep this mapping aligned with existing conf/config.json naming.
TOPIC_TO_SENSOR_NAME = {
    TOP_LIDAR_TOPIC: "lidar_mid",
    "/d1/front_right/point_cloud": "lidar_front",
    "/d1/front_left/point_cloud": "lidar_left",
    "/d1/rear_right/point_cloud": "lidar_right",
    "/d1/rear_left/point_cloud": "lidar_back",
}


def mat3_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [
            a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]
            for c in range(3)
        ]
        for r in range(3)
    ]


def mat3_vec3_mul(a: List[List[float]], v: List[float]) -> List[float]:
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]


def euler_deg_to_rot3(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[List[float]]:
    # Same convention as lidar_calibration_ws_v2/tools/manual_lidar_calibration_tool.py
    rr = math.radians(roll_deg)
    pr = math.radians(pitch_deg)
    yr = math.radians(yaw_deg)

    cx, sx = math.cos(rr), math.sin(rr)
    cy, sy = math.cos(pr), math.sin(pr)
    cz, sz = math.cos(yr), math.sin(yr)

    rx = [
        [1.0, 0.0, 0.0],
        [0.0, cx, -sx],
        [0.0, sx, cx],
    ]
    ry = [
        [cy, 0.0, sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0, cy],
    ]
    rz = [
        [cz, -sz, 0.0],
        [sz, cz, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return mat3_mul(mat3_mul(rz, ry), rx)


def build_transform(entry: Dict[str, float]) -> Tuple[List[List[float]], List[float]]:
    r = euler_deg_to_rot3(
        float(entry.get("roll_deg", 0.0)),
        float(entry.get("pitch_deg", 0.0)),
        float(entry.get("yaw_deg", 0.0)),
    )
    t = [
        float(entry.get("x", 0.0)),
        float(entry.get("y", 0.0)),
        float(entry.get("z", 0.0)),
    ]
    return r, t


def compose_transform(
    first: Tuple[List[List[float]], List[float]],
    second: Tuple[List[List[float]], List[float]],
) -> Tuple[List[List[float]], List[float]]:
    # If p_b = R1*p_a + t1 and p_c = R2*p_b + t2
    # then p_c = (R2*R1)*p_a + (R2*t1 + t2)
    r1, t1 = first
    r2, t2 = second
    r = mat3_mul(r2, r1)
    rt1 = mat3_vec3_mul(r2, t1)
    t = [rt1[0] + t2[0], rt1[1] + t2[1], rt1[2] + t2[2]]
    return r, t


def flatten_row_major(r: List[List[float]]) -> List[float]:
    return [r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]]


def round_list(values: List[float], ndigits: int) -> List[float]:
    return [round(float(v), ndigits) for v in values]


def default_sensor_template() -> Dict:
    return {
        "channel": "",
        "distortion": [],
        "image_size": [0, 0],
        "intrinsic": [],
        "modality": "lidar",
        "rotation": [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        "target": "asensing",
        "translation": [0.0, 0.0, 0.0],
        "undistort_distortion": [],
        "undistort_intrinsic": [],
    }


def write_json(path: Path, data: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
        f.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert lidar calibration res.json to multi_lidar_splicing conf")
    parser.add_argument(
        "--res-json",
        default="/home/lenovo/project/lidar_calibration_ws_v2/tools/res.json",
        help="Path to calibration result json",
    )
    parser.add_argument(
        "--conf-dir",
        default="/home/lenovo/project/legionclaw/modules/perception/lidar/multi_lidar_splicing/bin/conf",
        help="Output conf directory (contains config.json and sensors/)",
    )
    parser.add_argument(
        "--float-precision",
        type=int,
        default=9,
        help="Decimal precision for rotation/translation output",
    )
    args = parser.parse_args()

    res_path = Path(args.res_json)
    conf_dir = Path(args.conf_dir)
    config_path = conf_dir / "config.json"
    sensors_dir = conf_dir / "sensors"

    if not res_path.exists():
        raise FileNotFoundError(f"res.json not found: {res_path}")

    with res_path.open("r", encoding="utf-8") as f:
        res_data = json.load(f)

    if TOP_LIDAR_TOPIC not in res_data:
        raise KeyError(f"Missing required top lidar topic: {TOP_LIDAR_TOPIC}")

    tf_top_to_vehicle = build_transform(res_data[TOP_LIDAR_TOPIC])

    generated_topics = []
    for topic, sensor_name in TOPIC_TO_SENSOR_NAME.items():
        if topic not in res_data:
            raise KeyError(f"Missing topic in res.json: {topic}")

        if topic == TOP_LIDAR_TOPIC:
            tf_sensor_to_vehicle = tf_top_to_vehicle
        else:
            tf_sensor_to_top = build_transform(res_data[topic])
            tf_sensor_to_vehicle = compose_transform(tf_sensor_to_top, tf_top_to_vehicle)

        r_sensor_to_vehicle, t_sensor_to_vehicle = tf_sensor_to_vehicle

        sensor_path = sensors_dir / f"{sensor_name}.json"
        sensor_obj = default_sensor_template()
        if sensor_path.exists():
            with sensor_path.open("r", encoding="utf-8") as f:
                try:
                    existing = json.load(f)
                    if isinstance(existing, dict):
                        sensor_obj.update(existing)
                except json.JSONDecodeError:
                    pass

        sensor_obj["channel"] = topic
        sensor_obj["rotation"] = round_list(flatten_row_major(r_sensor_to_vehicle), args.float_precision)
        sensor_obj["translation"] = round_list(t_sensor_to_vehicle, args.float_precision)

        write_json(sensor_path, sensor_obj)
        generated_topics.append((sensor_name, topic, sensor_path))

    config_obj = {
        "calibration_params_path": {
            "lidar_front": "./conf/sensors/lidar_front.json",
            "lidar_mid": "./conf/sensors/lidar_mid.json",
            "lidar_left": "./conf/sensors/lidar_left.json",
            "lidar_right": "./conf/sensors/lidar_right.json",
            "lidar_back": "./conf/sensors/lidar_back.json",
        },
        "frame_id": "map",
        "publish_topic": "/rslidar_points",
    }

    if config_path.exists():
        with config_path.open("r", encoding="utf-8") as f:
            try:
                existing_config = json.load(f)
                if isinstance(existing_config, dict):
                    if "frame_id" in existing_config:
                        config_obj["frame_id"] = existing_config["frame_id"]
                    if "publish_topic" in existing_config:
                        config_obj["publish_topic"] = existing_config["publish_topic"]
            except json.JSONDecodeError:
                pass

    write_json(config_path, config_obj)

    print(f"Converted: {res_path}")
    print(f"Written config: {config_path}")
    print("Written sensors:")
    for sensor_name, topic, sensor_path in generated_topics:
        print(f"  - {sensor_name:<11} <- {topic} ({sensor_path})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
