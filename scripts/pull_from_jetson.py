#!/usr/bin/env python3
import os
import pathlib
import sys

import paramiko


JETSON_HOST = os.environ.get("JETSON_HOST", "10.0.30.108")
JETSON_USER = os.environ.get("JETSON_USER", "jetson2")
JETSON_PASSWORD = os.environ.get("JETSON_PASSWORD", "152535")
PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


FILES_TO_PULL = {
    "/home/jetson2/code/start_real_px4_mid360_fastlio.sh":
        PROJECT_ROOT / "docs" / "start_real_px4_mid360_fastlio.sh.jetson_copy",
    "/home/jetson2/code/run_real_mid360_lio.launch":
        PROJECT_ROOT / "docs" / "run_real_mid360_lio.launch.jetson_copy",
    "/home/jetson2/docker/ros_root/catkin_ws/src/px4_realflight_tools/scripts/odom_to_pose.py":
        PROJECT_ROOT / "scripts" / "odom_to_pose.py",
    "/home/jetson2/docker/ros_root/catkin_ws/src/Diff-Planner-PX4/src/diff_planner/plan_manage/launch/exp/run_real_mid360_lio.launch":
        PROJECT_ROOT / "third_party" / "Diff-Planner-PX4" / "src" / "diff_planner" / "plan_manage" / "launch" / "exp" / "run_real_mid360_lio.launch",
}


def main() -> int:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=JETSON_HOST,
        username=JETSON_USER,
        password=JETSON_PASSWORD,
        timeout=20,
    )

    sftp = client.open_sftp()
    try:
        for remote, local in FILES_TO_PULL.items():
            local.parent.mkdir(parents=True, exist_ok=True)
            sftp.get(remote, str(local))
            print(f"pulled: {remote} -> {local}")
    finally:
        sftp.close()
        client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
