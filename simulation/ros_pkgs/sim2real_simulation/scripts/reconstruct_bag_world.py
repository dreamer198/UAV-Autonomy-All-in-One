#!/usr/bin/env python3
"""Reconstruct a Gazebo Classic world from a registered outdoor ROS bag.

The registered clouds are already expressed in the bag's world frame.  This
tool keeps repeatedly observed voxels around the recorded flight corridor and
turns their exposed faces into one static OBJ mesh.  Gazebo can then use the
same mesh for collision detection and MID-360 ray returns.
"""

import argparse
import csv
import html
import json
import math
import os
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag


FACE_DEFINITIONS = (
    ((-1, 0, 0), ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))),
    ((1, 0, 0), ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1))),
    ((0, -1, 0), ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1))),
    ((0, 1, 0), ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0))),
    ((0, 0, -1), ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0))),
    ((0, 0, 1), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))),
)


def positive_float(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


def nonnegative_float(value):
    value = float(value)
    if value < 0.0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return value


def positive_int(value):
    value = int(value)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a static Gazebo world from registered bag clouds."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--cloud-topic", default="/localization/cloud_registered"
    )
    parser.add_argument("--odom-topic", default="/localization/odom")
    parser.add_argument("--voxel-size", type=positive_float, default=0.14)
    parser.add_argument("--cloud-stride", type=positive_int, default=2)
    parser.add_argument("--min-observations", type=positive_int, default=2)
    parser.add_argument(
        "--corridor-radius",
        type=positive_float,
        default=7.0,
        help="Keep cloud points within this horizontal distance of the vehicle.",
    )
    parser.add_argument(
        "--self-filter-radius", type=nonnegative_float, default=0.35
    )
    parser.add_argument(
        "--obstacle-min-z",
        type=float,
        default=0.18,
        help="Points at or below this height are represented by the ground plane.",
    )
    parser.add_argument("--obstacle-max-z", type=float, default=3.2)
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument("--wind-speed", type=nonnegative_float, default=0.0)
    parser.add_argument(
        "--wind-direction",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(1.0, 0.0, 0.0),
    )
    return parser.parse_args()


def point_field_array(msg, name):
    field = next((item for item in msg.fields if item.name == name), None)
    if field is None:
        raise ValueError("PointCloud2 is missing required field: {}".format(name))
    byte_order = ">" if msg.is_bigendian else "<"
    return np.ndarray(
        msg.width * msg.height,
        dtype=byte_order + "f4",
        buffer=msg.data,
        offset=field.offset,
        strides=(msg.point_step,),
    )


def xyz_array(msg):
    return np.column_stack(
        (
            point_field_array(msg, "x"),
            point_field_array(msg, "y"),
            point_field_array(msg, "z"),
        )
    )


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def pose_record(msg, relative_time):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return {
        "time": relative_time,
        "x": position.x,
        "y": position.y,
        "z": position.z,
        "qx": orientation.x,
        "qy": orientation.y,
        "qz": orientation.z,
        "qw": orientation.w,
        "yaw": yaw_from_quaternion(orientation),
    }


def collect_voxels(args):
    voxel_counts = defaultdict(int)
    odometry = []
    latest_position = None
    cloud_count = 0
    processed_cloud_count = 0
    accepted_point_count = 0
    unique_scan_voxel_count = 0

    with rosbag.Bag(str(args.bag), "r") as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        topics = (args.odom_topic, args.cloud_topic)
        for topic, msg, stamp in bag.read_messages(topics=topics):
            relative_time = stamp.to_sec() - start_time
            if topic == args.odom_topic:
                record = pose_record(msg, relative_time)
                odometry.append(record)
                latest_position = np.array(
                    (record["x"], record["y"], record["z"]),
                    dtype=np.float32,
                )
                continue

            cloud_count += 1
            if latest_position is None:
                continue
            if (cloud_count - 1) % args.cloud_stride != 0:
                continue

            points = xyz_array(msg)
            valid = np.isfinite(points).all(axis=1)
            valid &= points[:, 2] > args.obstacle_min_z
            valid &= points[:, 2] < args.obstacle_max_z

            horizontal = points[:, :2] - latest_position[:2]
            valid &= np.einsum("ij,ij->i", horizontal, horizontal) < (
                args.corridor_radius * args.corridor_radius
            )
            if args.self_filter_radius > 0.0:
                relative = points - latest_position
                valid &= np.einsum("ij,ij->i", relative, relative) > (
                    args.self_filter_radius * args.self_filter_radius
                )

            points = points[valid]
            processed_cloud_count += 1
            accepted_point_count += points.shape[0]
            if points.size == 0:
                continue

            scan_voxels = np.unique(
                np.floor(points / args.voxel_size).astype(np.int32),
                axis=0,
            )
            unique_scan_voxel_count += scan_voxels.shape[0]
            for x_index, y_index, z_index in scan_voxels:
                voxel_counts[
                    (int(x_index), int(y_index), int(z_index))
                ] += 1

    retained = {
        key
        for key, count in voxel_counts.items()
        if count >= args.min_observations
    }
    if not odometry:
        raise RuntimeError(
            "No odometry messages found on {}".format(args.odom_topic)
        )
    if not retained:
        raise RuntimeError(
            "No voxels survived filtering; lower --min-observations or "
            "check the cloud/odometry topics."
        )

    stats = {
        "bag_start_time": start_time,
        "bag_end_time": end_time,
        "duration": end_time - start_time,
        "cloud_messages": cloud_count,
        "processed_cloud_messages": processed_cloud_count,
        "accepted_cloud_points": accepted_point_count,
        "unique_voxels_across_scans": unique_scan_voxel_count,
        "candidate_voxels": len(voxel_counts),
        "retained_voxels": len(retained),
    }
    return retained, odometry, stats


def exposed_faces(voxels):
    for voxel in voxels:
        for neighbor_offset, corners in FACE_DEFINITIONS:
            neighbor = (
                voxel[0] + neighbor_offset[0],
                voxel[1] + neighbor_offset[1],
                voxel[2] + neighbor_offset[2],
            )
            if neighbor not in voxels:
                yield voxel, corners


def write_obj(path, voxels, voxel_size):
    vertex_indices = {}
    face_count = 0
    for voxel, corners in exposed_faces(voxels):
        face_count += 1
        for corner in corners:
            vertex = (
                voxel[0] + corner[0],
                voxel[1] + corner[1],
                voxel[2] + corner[2],
            )
            if vertex not in vertex_indices:
                vertex_indices[vertex] = len(vertex_indices) + 1

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output:
        output.write("# Reconstructed from registered ROS bag point clouds\n")
        output.write("mtllib scene.mtl\n")
        output.write("o bag_reconstructed_environment\n")
        for vertex in vertex_indices:
            output.write(
                "v {:.6f} {:.6f} {:.6f}\n".format(
                    vertex[0] * voxel_size,
                    vertex[1] * voxel_size,
                    vertex[2] * voxel_size,
                )
            )
        output.write("usemtl outdoor_obstacle\n")
        for voxel, corners in exposed_faces(voxels):
            indices = []
            for corner in corners:
                vertex = (
                    voxel[0] + corner[0],
                    voxel[1] + corner[1],
                    voxel[2] + corner[2],
                )
                indices.append(vertex_indices[vertex])
            output.write("f {} {} {} {}\n".format(*indices))

    material_path = path.with_name("scene.mtl")
    with material_path.open("w", encoding="utf-8") as output:
        output.write("newmtl outdoor_obstacle\n")
        output.write("Ka 0.12 0.18 0.10\n")
        output.write("Kd 0.28 0.45 0.22\n")
        output.write("Ks 0.02 0.02 0.02\n")
        output.write("d 1.0\n")
        output.write("illum 2\n")

    return len(vertex_indices), face_count


def normalized_direction(values):
    direction = np.asarray(values, dtype=np.float64)
    norm = np.linalg.norm(direction)
    if norm <= 1.0e-9:
        raise ValueError("--wind-direction must not be the zero vector")
    direction /= norm
    return direction


def write_world(path, mesh_path, args, bounds):
    direction = normalized_direction(args.wind_direction)
    min_bound, max_bound = bounds
    center_x = 0.5 * (min_bound[0] + max_bound[0])
    center_y = 0.5 * (min_bound[1] + max_bound[1])
    ground_x = max(100.0, max_bound[0] - min_bound[0] + 30.0)
    ground_y = max(100.0, max_bound[1] - min_bound[1] + 30.0)
    mesh_uri = "file://" + html.escape(str(mesh_path.resolve()), quote=True)

    contents = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="se3_outdoor_bag_reconstruction">
    <light name="sun" type="directional">
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.85 0.85 0.82 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.45 0.15 -0.88</direction>
      <cast_shadows>true</cast_shadows>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <pose>{center_x:.6f} {center_y:.6f} {ground_z:.6f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_x:.3f} {ground_y:.3f}</size>
            </plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_x:.3f} {ground_y:.3f}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.18 0.22 0.14 1</ambient>
            <diffuse>0.32 0.38 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="bag_reconstructed_environment">
      <static>true</static>
      <link name="environment_link">
        <collision name="environment_collision">
          <geometry>
            <mesh><uri>{mesh_uri}</uri></mesh>
          </geometry>
          <max_contacts>20</max_contacts>
        </collision>
        <visual name="environment_visual">
          <cast_shadows>true</cast_shadows>
          <geometry>
            <mesh><uri>{mesh_uri}</uri></mesh>
          </geometry>
        </visual>
      </link>
    </model>
    <plugin name="wind_plugin" filename="libgazebo_wind_plugin.so">
      <frameId>base_link</frameId>
      <robotNamespace/>
      <windVelocityMean>{wind_speed:.6f}</windVelocityMean>
      <windVelocityMax>{wind_speed:.6f}</windVelocityMax>
      <windVelocityVariance>0</windVelocityVariance>
      <windDirectionMean>{wind_x:.9f} {wind_y:.9f} {wind_z:.9f}</windDirectionMean>
      <windDirectionVariance>0</windDirectionVariance>
      <windGustStart>0</windGustStart>
      <windGustDuration>0</windGustDuration>
      <windGustVelocityMean>0</windGustVelocityMean>
      <windGustVelocityMax>0</windGustVelocityMax>
      <windGustVelocityVariance>0</windGustVelocityVariance>
      <windGustDirectionMean>1 0 0</windGustDirectionMean>
      <windGustDirectionVariance>0</windGustDirectionVariance>
      <windPubTopic>world_wind</windPubTopic>
    </plugin>
    <gravity>0 0 -9.8066</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <ode>
        <solver><type>quick</type><iters>20</iters><sor>1.3</sor></solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    <scene>
      <ambient>0.45 0.45 0.42 1</ambient>
      <background>0.72 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
  </world>
</sdf>
""".format(
        center_x=center_x,
        center_y=center_y,
        ground_z=args.ground_z,
        ground_x=ground_x,
        ground_y=ground_y,
        mesh_uri=mesh_uri,
        wind_speed=args.wind_speed,
        wind_x=direction[0],
        wind_y=direction[1],
        wind_z=direction[2],
    )
    path.write_text(contents, encoding="utf-8")


def write_trajectory_csv(path, odometry):
    fieldnames = (
        "time",
        "x",
        "y",
        "z",
        "qx",
        "qy",
        "qz",
        "qw",
        "yaw",
    )
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(odometry)


def main():
    args = parse_args()
    args.bag = args.bag.expanduser().resolve()
    args.output_dir = args.output_dir.expanduser().resolve()
    if not args.bag.is_file():
        raise FileNotFoundError(args.bag)
    if args.obstacle_max_z <= args.obstacle_min_z:
        raise ValueError("--obstacle-max-z must be greater than --obstacle-min-z")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir = args.output_dir / "meshes"
    mesh_path = mesh_dir / "scene.obj"
    world_path = args.output_dir / "se3_outdoor_reconstruction.world"

    voxels, odometry, stats = collect_voxels(args)
    vertex_count, face_count = write_obj(
        mesh_path, voxels, args.voxel_size
    )

    voxel_array = np.asarray(tuple(voxels), dtype=np.int32)
    min_bound = voxel_array.min(axis=0).astype(np.float64) * args.voxel_size
    max_bound = (
        voxel_array.max(axis=0).astype(np.float64) + 1.0
    ) * args.voxel_size
    write_world(world_path, mesh_path, args, (min_bound, max_bound))
    write_trajectory_csv(args.output_dir / "recorded_trajectory.csv", odometry)

    metadata = {
        "source_bag": str(args.bag),
        "source_bag_size": os.path.getsize(args.bag),
        "cloud_topic": args.cloud_topic,
        "odom_topic": args.odom_topic,
        "voxel_size": args.voxel_size,
        "cloud_stride": args.cloud_stride,
        "min_observations": args.min_observations,
        "corridor_radius": args.corridor_radius,
        "self_filter_radius": args.self_filter_radius,
        "obstacle_min_z": args.obstacle_min_z,
        "obstacle_max_z": args.obstacle_max_z,
        "ground_z": args.ground_z,
        "wind_speed": args.wind_speed,
        "wind_direction": normalized_direction(
            args.wind_direction
        ).tolist(),
        "world_file": str(world_path),
        "mesh_file": str(mesh_path),
        "mesh_vertices": vertex_count,
        "mesh_quads": face_count,
        "bounds_min": min_bound.tolist(),
        "bounds_max": max_bound.tolist(),
        "initial_pose": odometry[0],
        "statistics": stats,
    }
    (args.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    print("Gazebo reconstruction complete")
    print("  world: {}".format(world_path))
    print("  mesh: {}".format(mesh_path))
    print(
        "  voxels/vertices/quads: {}/{}/{}".format(
            len(voxels), vertex_count, face_count
        )
    )
    print(
        "  bounds: {} -> {}".format(
            np.array2string(min_bound, precision=3),
            np.array2string(max_bound, precision=3),
        )
    )


if __name__ == "__main__":
    main()
