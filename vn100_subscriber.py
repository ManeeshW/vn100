#!/usr/bin/env python3
"""
VN100 IMU Zenoh subscriber.

Reads the Zenoh topic from config.cfg (IMU.zenoh_topic) and subscribes
to incoming IMU messages published by the vn100 C++ driver.

Usage:
    python3 vn100_subscriber.py [config_path]

Each message is a JSON string with the following fields:
    timestamp    - Unix epoch time in seconds (float)
    ypr_deg      - [yaw, pitch, roll] in degrees
    quat         - [x, y, z, w] quaternion
    ang_rate     - [x, y, z] angular rate in rad/s
    accel        - [x, y, z] linear acceleration in m/s²
"""

import sys
import time
import json
import zenoh


DEFAULT_CONFIG = "config.cfg"
DEFAULT_TOPIC = "fdcl/imu"


def parse_config(config_path: str) -> dict:
    cfg = {
        "zenoh_topic": "fdcl/imu/",
        "zenoh_publish_rate": 200,
        "frequency": 200,
    }
    try:
        with open(config_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    continue
                key, _, val = line.partition("=")
                key, val = key.strip(), val.strip()
                if key == "IMU.zenoh_topic":
                    cfg["zenoh_topic"] = val
                elif key == "IMU.zenoh_publish_rate":
                    cfg["zenoh_publish_rate"] = int(val)
                elif key == "IMU.frequency":
                    cfg["frequency"] = int(val)
    except FileNotFoundError:
        print(f"[warn] Config '{config_path}' not found, using defaults.")
    return cfg


_last_ts: float = 0.0
_msg_count: int = 0
_hz_window_start: float = time.monotonic()
_hz_count: int = 0


def on_sample(sample) -> None:
    global _last_ts, _msg_count, _hz_window_start, _hz_count

    try:
        payload = bytes(sample.payload).decode()
        data = json.loads(payload)
    except Exception as e:
        print(f"[error] Failed to decode message: {e}")
        return

    _msg_count += 1
    _hz_count += 1

    now = time.monotonic()
    elapsed = now - _hz_window_start
    if elapsed >= 1.0:
        hz = _hz_count / elapsed
        _hz_count = 0
        _hz_window_start = now
    else:
        hz = None

    ts = data.get("timestamp", 0.0)
    ypr = data.get("ypr_deg", [0, 0, 0])
    quat = data.get("quat", [0, 0, 0, 1])
    ang = data.get("ang_rate", [0, 0, 0])
    acc = data.get("accel", [0, 0, 0])

    if _msg_count == 1:
        print()  # blank line before first data block

    # move cursor up 6 lines after first message to overwrite in-place
    if _msg_count > 1:
        print("\033[6A", end="")

    hz_str = f"{hz:.1f} Hz" if hz is not None else "-- Hz"
    print(f"\033[2K--- IMU [{hz_str}]  msg #{_msg_count}  ts={ts:.3f}")
    print(f"\033[2K  YPR    (deg) : {ypr[0]:+9.4f}  {ypr[1]:+9.4f}  {ypr[2]:+9.4f}")
    print(f"\033[2K  Quat         : {quat[0]:+8.5f}  {quat[1]:+8.5f}  {quat[2]:+8.5f}  {quat[3]:+8.5f}")
    print(f"\033[2K  Ang Rate(r/s): {ang[0]:+9.4f}  {ang[1]:+9.4f}  {ang[2]:+9.4f}")
    print(f"\033[2K  Accel (m/s²) : {acc[0]:+9.4f}  {acc[1]:+9.4f}  {acc[2]:+9.4f}")
    print(f"\033[2K  Key expr     : {sample.key_expr}", flush=True)


def main() -> None:
    config_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG
    cfg = parse_config(config_path)

    topic = cfg["zenoh_topic"].rstrip("/")  # strip any accidental trailing slash

    print(f"Subscribing to Zenoh topic: '{topic}'")
    print(f"Config: {config_path}  |  sensor freq: {cfg['frequency']} Hz  "
          f"|  publish rate: {cfg['zenoh_publish_rate']} Hz")
    print("Press Ctrl+C to exit.\n")

    conf = zenoh.Config()
    with zenoh.open(conf) as session:
        with session.declare_subscriber(topic, on_sample):
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print(f"\n\nReceived {_msg_count} messages. Exiting.")


if __name__ == "__main__":
    main()
