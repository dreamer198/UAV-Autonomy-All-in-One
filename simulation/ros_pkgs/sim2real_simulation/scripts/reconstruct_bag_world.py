#!/usr/bin/env python3
"""Reconstruct a clean Gazebo Classic forest from a registered outdoor bag.

The registered clouds are already expressed in the bag's world frame.  The
default ``forest`` geometry mode projects repeatedly observed voxels into a
smoothed horizontal density map, detects stable local peaks, and represents
each peak as a trunk and a low-poly crown.  This removes isolated LiDAR
returns instead of turning every return into a floating cube.  The legacy
``voxels`` mode remains available for diagnostics.
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
POINT_FIELD_DTYPES = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


def finite_float(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("must be finite")
    return value


def positive_float(value):
    value = finite_float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


def nonnegative_float(value):
    value = finite_float(value)
    if value < 0.0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return value


def positive_int(value):
    value = int(value)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def unit_interval(value):
    value = finite_float(value)
    if not 0.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError("must be between zero and one")
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
        type=finite_float,
        default=0.18,
        help="Points at or below this height are represented by the ground plane.",
    )
    parser.add_argument("--obstacle-max-z", type=finite_float, default=3.2)
    parser.add_argument("--ground-z", type=finite_float, default=0.0)
    parser.add_argument(
        "--geometry-mode",
        choices=("forest", "voxels"),
        default="forest",
        help="Generate semantic trees by default; voxels preserves the legacy mesh.",
    )
    parser.add_argument("--tree-grid-size", type=positive_float, default=0.28)
    parser.add_argument(
        "--tree-smoothing-radius", type=positive_float, default=0.40
    )
    parser.add_argument(
        "--tree-min-spacing", type=positive_float, default=1.20
    )
    parser.add_argument(
        "--tree-density-quantile", type=unit_interval, default=0.80
    )
    parser.add_argument("--tree-trunk-radius", type=positive_float, default=0.19)
    parser.add_argument("--tree-crown-radius", type=positive_float, default=0.78)
    parser.add_argument("--tree-min-height", type=positive_float, default=2.50)
    parser.add_argument("--tree-max-height", type=positive_float, default=4.20)
    parser.add_argument("--forest-min-x", type=finite_float)
    parser.add_argument("--forest-max-x", type=finite_float)
    parser.add_argument("--forest-min-y", type=finite_float)
    parser.add_argument("--forest-max-y", type=finite_float)
    parser.add_argument(
        "--forest-fill-spacing",
        type=positive_float,
        default=2.0,
        help="Maximum distance from an empty rectangle location to a tree.",
    )
    parser.add_argument(
        "--forest-path-clearance", type=nonnegative_float, default=0.65
    )
    parser.add_argument(
        "--forest-corner-clearance", type=nonnegative_float, default=1.50
    )
    parser.add_argument("--forest-seed", type=positive_int, default=151241)
    parser.add_argument(
        "--mesh-uri",
        default="",
        help="Optional portable URI written into the world instead of the output path.",
    )
    parser.add_argument("--wind-speed", type=nonnegative_float, default=0.0)
    parser.add_argument(
        "--wind-direction",
        nargs=3,
        type=finite_float,
        metavar=("X", "Y", "Z"),
        default=(1.0, 0.0, 0.0),
    )
    return parser.parse_args()


def point_field_array(msg, name):
    field = next((item for item in msg.fields if item.name == name), None)
    if field is None:
        raise ValueError("PointCloud2 is missing required field: {}".format(name))
    datatype = POINT_FIELD_DTYPES.get(int(field.datatype))
    if datatype is None:
        raise ValueError(
            "PointCloud2 field '{}' has unsupported datatype {}".format(
                name, field.datatype
            )
        )
    if int(field.count) < 1:
        raise ValueError(
            "PointCloud2 field '{}' has invalid count {}".format(
                name, field.count
            )
        )
    width = int(msg.width)
    height = int(msg.height)
    point_step = int(msg.point_step)
    row_step = int(msg.row_step)
    if width < 0 or height <= 0 or point_step <= 0:
        raise ValueError("PointCloud2 dimensions or point_step are invalid")
    if row_step < width * point_step:
        raise ValueError("PointCloud2 row_step is shorter than one point row")
    byte_order = ">" if msg.is_bigendian else "<"
    dtype = np.dtype(byte_order + datatype)
    if width == 0:
        return np.empty(0, dtype=dtype)
    if int(field.offset) < 0 or int(field.offset) + dtype.itemsize > point_step:
        raise ValueError(
            "PointCloud2 field '{}' does not fit inside point_step".format(name)
        )
    required_bytes = (
        (height - 1) * row_step
        + max(0, width - 1) * point_step
        + int(field.offset)
        + dtype.itemsize
    )
    if required_bytes > len(msg.data):
        raise ValueError("PointCloud2 data is shorter than its declared layout")
    array = np.ndarray(
        (height, width),
        dtype=dtype,
        buffer=msg.data,
        offset=field.offset,
        strides=(row_step, point_step),
    )
    return np.asarray(array).reshape(-1)


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
    values = (
        relative_time,
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Odometry contains a non-finite pose or timestamp")
    quaternion_norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if quaternion_norm <= 1e-9:
        raise ValueError("Odometry contains an invalid zero-norm orientation")
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


def write_voxel_obj(path, voxels, voxel_size):
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


def _gaussian_kernel(sigma_cells):
    radius = max(1, int(math.ceil(3.0 * sigma_cells)))
    offsets = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-0.5 * np.square(offsets / sigma_cells))
    return kernel / kernel.sum()


def _smooth_density(grid, sigma_cells):
    kernel = _gaussian_kernel(sigma_cells)
    padding = len(kernel) // 2

    def convolve(values):
        padded = np.pad(values, (padding, padding), mode="constant")
        return np.convolve(padded, kernel, mode="valid")

    smoothed = np.apply_along_axis(
        convolve, 0, grid
    )
    return np.apply_along_axis(convolve, 1, smoothed)


def _local_maximum_mask(values, radius):
    padded = np.pad(
        values,
        ((radius, radius), (radius, radius)),
        mode="constant",
        constant_values=-np.inf,
    )
    maxima = np.full(values.shape, -np.inf, dtype=np.float64)
    for x_offset in range(2 * radius + 1):
        for y_offset in range(2 * radius + 1):
            maxima = np.maximum(
                maxima,
                padded[
                    x_offset : x_offset + values.shape[0],
                    y_offset : y_offset + values.shape[1],
                ],
            )
    return values >= maxima - 1.0e-12


def _forest_bounds(args):
    values = (
        args.forest_min_x,
        args.forest_max_x,
        args.forest_min_y,
        args.forest_max_y,
    )
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError(
            "forest bounds require --forest-min-x, --forest-max-x, "
            "--forest-min-y and --forest-max-y together"
        )
    if args.forest_max_x <= args.forest_min_x:
        raise ValueError("--forest-max-x must be greater than --forest-min-x")
    if args.forest_max_y <= args.forest_min_y:
        raise ValueError("--forest-max-y must be greater than --forest-min-y")
    return values


def _minimum_distances(points, references):
    if not len(points):
        return np.empty(0, dtype=np.float64)
    if not len(references):
        return np.full(len(points), np.inf, dtype=np.float64)
    result = np.full(len(points), np.inf, dtype=np.float64)
    for reference in references:
        delta = points - reference
        result = np.minimum(result, np.einsum("ij,ij->i", delta, delta))
    return np.sqrt(result)


def _complete_rectangle(selected, odometry, args, bounds):
    minimum_x, maximum_x, minimum_y, maximum_y = bounds
    edge_margin = min(
        args.tree_crown_radius * 0.65,
        0.45 * min(maximum_x - minimum_x, maximum_y - minimum_y),
    )
    corners = np.asarray(
        ((minimum_x, maximum_y), (maximum_x, minimum_y)),
        dtype=np.float64,
    )
    path = np.asarray(
        [(record["x"], record["y"]) for record in odometry[::5]],
        dtype=np.float64,
    )
    extension_length = float(np.linalg.norm(corners[1] - path[-1]))
    extension_samples = max(2, int(math.ceil(extension_length / 0.20)) + 1)
    extension = np.linspace(path[-1], corners[1], extension_samples)
    path = np.vstack((corners[0], path, extension[1:]))

    def allowed(points):
        mask = (
            (points[:, 0] >= minimum_x + edge_margin)
            & (points[:, 0] <= maximum_x - edge_margin)
            & (points[:, 1] >= minimum_y + edge_margin)
            & (points[:, 1] <= maximum_y - edge_margin)
        )
        if args.forest_path_clearance > 0.0:
            mask &= (
                _minimum_distances(points, path)
                >= args.forest_path_clearance
            )
        if args.forest_corner_clearance > 0.0:
            mask &= (
                _minimum_distances(points, corners)
                >= args.forest_corner_clearance
            )
        return mask

    measured_points = np.asarray(
        [(tree["x"], tree["y"]) for tree in selected], dtype=np.float64
    ).reshape((-1, 2))
    measured_mask = allowed(measured_points)
    selected = [
        dict(tree, source="measured_density")
        for tree, keep in zip(selected, measured_mask)
        if keep
    ]

    candidate_step = min(0.42, args.forest_fill_spacing / 4.0)
    x_values = np.arange(
        minimum_x + edge_margin,
        maximum_x - edge_margin + 0.5 * candidate_step,
        candidate_step,
    )
    y_values = np.arange(
        minimum_y + edge_margin,
        maximum_y - edge_margin + 0.5 * candidate_step,
        candidate_step,
    )
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="ij")
    candidates = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    rng = np.random.default_rng(args.forest_seed)
    candidates += rng.uniform(
        -0.20 * candidate_step,
        0.20 * candidate_step,
        size=candidates.shape,
    )
    candidates = candidates[allowed(candidates)]

    tree_points = np.asarray(
        [(tree["x"], tree["y"]) for tree in selected], dtype=np.float64
    ).reshape((-1, 2))
    nearest = _minimum_distances(candidates, tree_points)
    while len(candidates):
        index = int(np.argmax(nearest))
        if nearest[index] <= args.forest_fill_spacing:
            break
        position = candidates[index]
        selected.append(
            {
                "x": float(position[0]),
                "y": float(position[1]),
                "density_score": 0.0,
                "source": "rectangle_fill",
            }
        )
        delta = candidates - position
        nearest = np.minimum(
            nearest, np.sqrt(np.einsum("ij,ij->i", delta, delta))
        )

    return selected, {
        "bounds": {
            "min_x": minimum_x,
            "max_x": maximum_x,
            "min_y": minimum_y,
            "max_y": maximum_y,
        },
        "edge_margin": edge_margin,
        "path_clearance": args.forest_path_clearance,
        "corner_clearance": args.forest_corner_clearance,
        "fill_spacing": args.forest_fill_spacing,
        "measured_tree_count": sum(
            tree["source"] == "measured_density" for tree in selected
        ),
        "filled_tree_count": sum(
            tree["source"] == "rectangle_fill" for tree in selected
        ),
    }


def detect_trees(voxels, odometry, args):
    voxel_array = np.asarray(tuple(voxels), dtype=np.int32)
    centers = (
        voxel_array.astype(np.float64) + np.array((0.5, 0.5, 0.5))
    ) * args.voxel_size
    cell = args.tree_grid_size
    cell_indices = np.floor(centers[:, :2] / cell).astype(np.int32)
    minimum = cell_indices.min(axis=0)
    maximum = cell_indices.max(axis=0)
    density = np.zeros(
        tuple((maximum - minimum + 1).tolist()), dtype=np.float64
    )
    shifted = cell_indices - minimum
    np.add.at(density, (shifted[:, 0], shifted[:, 1]), 1.0)

    sigma_cells = args.tree_smoothing_radius / cell
    score = _smooth_density(density, sigma_cells)
    occupied_scores = score[score > 1.0e-9]
    threshold = float(
        np.quantile(occupied_scores, args.tree_density_quantile)
    )
    peak_radius = max(1, int(round(args.tree_min_spacing / cell)))
    peak_mask = _local_maximum_mask(score, peak_radius)
    candidate_indices = np.argwhere(peak_mask & (score >= threshold))
    candidates = []
    for grid_index in candidate_indices:
        position = (grid_index + minimum + 0.5) * cell
        candidates.append(
            (
                -float(score[tuple(grid_index)]),
                float(position[0]),
                float(position[1]),
            )
        )
    candidates.sort()

    selected = []
    minimum_distance_squared = args.tree_min_spacing ** 2
    for negative_score, x_position, y_position in candidates:
        if any(
            (x_position - tree["x"]) ** 2 + (y_position - tree["y"]) ** 2
            < minimum_distance_squared
            for tree in selected
        ):
            continue
        selected.append(
            {
                "x": x_position,
                "y": y_position,
                "density_score": -negative_score,
                "source": "measured_density",
            }
        )

    if not selected:
        raise RuntimeError(
            "No tree density peaks survived filtering; lower "
            "--tree-density-quantile or --tree-min-spacing."
        )

    rectangle_stats = {}
    bounds = _forest_bounds(args)
    if bounds is not None:
        selected, rectangle_stats = _complete_rectangle(
            selected, odometry, args, bounds
        )

    measured_scores = [
        tree["density_score"]
        for tree in selected
        if tree["source"] == "measured_density"
    ]
    score_min = min(measured_scores, default=threshold)
    score_max = max(measured_scores, default=threshold + 1.0)
    score_span = max(1.0e-9, score_max - score_min)
    evidence_radius = max(args.tree_crown_radius, args.tree_min_spacing * 0.5)
    size_rng = np.random.default_rng(args.forest_seed + 1)
    for tree in sorted(selected, key=lambda item: (item["x"], item["y"])):
        distance_squared = np.square(
            centers[:, 0] - tree["x"]
        ) + np.square(centers[:, 1] - tree["y"])
        local_heights = centers[
            distance_squared <= evidence_radius ** 2, 2
        ]
        if local_heights.size:
            tree_height = float(
                np.quantile(local_heights, 0.98) + 0.5 * args.voxel_size
            )
        else:
            tree_height = args.tree_min_height
        if tree["source"] == "measured_density":
            strength = max(
                0.0,
                min(
                    1.0,
                    (tree["density_score"] - score_min) / score_span,
                ),
            )
        else:
            strength = float(size_rng.uniform(0.20, 0.65))
            tree["density_score"] = threshold * (0.85 + 0.30 * strength)
        height_noise = float(size_rng.beta(2.0, 2.0))
        inferred_height = args.tree_min_height + (
            args.tree_max_height - args.tree_min_height
        ) * (0.10 + 0.72 * height_noise + 0.18 * strength)
        tree["height"] = min(
            args.tree_max_height,
            max(args.tree_min_height, tree_height, inferred_height),
        )
        trunk_noise = float(size_rng.uniform(0.0, 1.0))
        tree["trunk_radius"] = args.tree_trunk_radius * (
            0.68 + 0.38 * strength + 0.38 * trunk_noise
        )
        tree["_size_strength"] = strength
        tree["_crown_noise"] = float(size_rng.uniform(0.0, 1.0))
        tree["_crown_shape"] = float(size_rng.uniform(0.85, 1.25))

    for tree in selected:
        nearest = min(
            (
                math.hypot(tree["x"] - other["x"], tree["y"] - other["y"])
                for other in selected
                if other is not tree
            ),
            default=2.0 * args.tree_crown_radius,
        )
        strength = tree.pop("_size_strength")
        crown_noise = tree.pop("_crown_noise")
        crown_shape = tree.pop("_crown_shape")
        desired_radius = args.tree_crown_radius * (
            0.62 + 0.36 * strength + 0.40 * crown_noise
        )
        tree["crown_radius"] = max(
            0.42, min(desired_radius, 0.44 * nearest)
        )
        if bounds is not None:
            edge_distance = min(
                tree["x"] - bounds[0],
                bounds[1] - tree["x"],
                tree["y"] - bounds[2],
                bounds[3] - tree["y"],
            )
            tree["crown_radius"] = min(
                tree["crown_radius"], max(0.35, edge_distance - 0.04)
            )
        crown_height = max(
            1.05, 2.05 * tree["crown_radius"] * crown_shape
        )
        tree["crown_top"] = tree["height"]
        tree["crown_bottom"] = max(
            args.ground_z + 1.05,
            tree["crown_top"] - crown_height,
        )

    return selected, {
        "density_threshold": threshold,
        "density_score_min": score_min,
        "density_score_max": score_max,
        "density_grid_shape": list(density.shape),
        "rectangle_completion": rectangle_stats,
    }


def _append_cylinder(vertices, faces, x, y, bottom, top, radius, sides):
    start = len(vertices) + 1
    for z_position in (bottom, top):
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            vertices.append(
                (
                    x + radius * math.cos(angle),
                    y + radius * math.sin(angle),
                    z_position,
                )
            )
    for index in range(sides):
        following = (index + 1) % sides
        faces.append(
            (
                start + index,
                start + following,
                start + sides + following,
                start + sides + index,
            )
        )
    faces.append(tuple(start + index for index in reversed(range(sides))))
    faces.append(tuple(start + sides + index for index in range(sides)))


def _append_ellipsoid(
    vertices,
    faces,
    x,
    y,
    bottom,
    top,
    horizontal_radius,
    segments,
    rings,
):
    start = len(vertices) + 1
    center_z = 0.5 * (bottom + top)
    vertical_radius = 0.5 * (top - bottom)
    vertices.append((x, y, top))
    for ring in range(1, rings):
        phi = math.pi * ring / rings
        ring_radius = horizontal_radius * math.sin(phi)
        z_position = center_z + vertical_radius * math.cos(phi)
        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            vertices.append(
                (
                    x + ring_radius * math.cos(angle),
                    y + ring_radius * math.sin(angle),
                    z_position,
                )
            )
    bottom_index = len(vertices) + 1
    vertices.append((x, y, bottom))

    first_ring = start + 1
    for index in range(segments):
        following = (index + 1) % segments
        faces.append((start, first_ring + index, first_ring + following))
    for ring in range(rings - 2):
        current = first_ring + ring * segments
        following_ring = current + segments
        for index in range(segments):
            following = (index + 1) % segments
            faces.append(
                (
                    current + index,
                    following_ring + index,
                    following_ring + following,
                    current + following,
                )
            )
    last_ring = first_ring + (rings - 2) * segments
    for index in range(segments):
        following = (index + 1) % segments
        faces.append(
            (last_ring + following, last_ring + index, bottom_index)
        )


def write_forest_obj(path, trees, ground_z):
    vertices = []
    trunk_faces = []
    crown_faces = []
    for tree in trees:
        _append_cylinder(
            vertices,
            trunk_faces,
            tree["x"],
            tree["y"],
            ground_z,
            tree["height"],
            tree["trunk_radius"],
            sides=10,
        )
        _append_ellipsoid(
            vertices,
            crown_faces,
            tree["x"],
            tree["y"],
            tree["crown_bottom"],
            tree["crown_top"],
            tree["crown_radius"],
            segments=10,
            rings=5,
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output:
        output.write("# Semantic forest reconstructed from registered LiDAR density\n")
        output.write("mtllib scene.mtl\n")
        output.write("o reconstructed_tree_trunks_and_crowns\n")
        for vertex in vertices:
            output.write(
                "v {:.6f} {:.6f} {:.6f}\n".format(*vertex)
            )
        output.write("usemtl tree_trunk\n")
        for face in trunk_faces:
            output.write("f {}\n".format(" ".join(map(str, face))))
        output.write("usemtl tree_crown\n")
        for face in crown_faces:
            output.write("f {}\n".format(" ".join(map(str, face))))

    material_path = path.with_name("scene.mtl")
    with material_path.open("w", encoding="utf-8") as output:
        output.write("newmtl tree_trunk\n")
        output.write("Ka 0.16 0.09 0.035\n")
        output.write("Kd 0.34 0.18 0.07\n")
        output.write("Ks 0.01 0.01 0.01\n")
        output.write("d 1.0\n")
        output.write("illum 2\n\n")
        output.write("newmtl tree_crown\n")
        output.write("Ka 0.08 0.16 0.06\n")
        output.write("Kd 0.20 0.43 0.16\n")
        output.write("Ks 0.015 0.02 0.012\n")
        output.write("d 1.0\n")
        output.write("illum 2\n")

    return len(vertices), len(trunk_faces) + len(crown_faces)


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
    mesh_uri = args.mesh_uri or ("file://" + str(mesh_path.resolve()))
    mesh_uri = html.escape(mesh_uri, quote=True)

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
        writer = csv.DictWriter(
            output, fieldnames=fieldnames, lineterminator="\n"
        )
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
    if args.tree_max_height < args.tree_min_height:
        raise ValueError("--tree-max-height must not be below --tree-min-height")
    forest_bounds = _forest_bounds(args)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir = args.output_dir / "meshes"
    mesh_path = mesh_dir / "scene.obj"
    world_path = args.output_dir / "se3_outdoor_reconstruction.world"

    voxels, odometry, stats = collect_voxels(args)
    trees = []
    forest_stats = {}
    if args.geometry_mode == "forest":
        trees, forest_stats = detect_trees(voxels, odometry, args)
        vertex_count, face_count = write_forest_obj(
            mesh_path, trees, args.ground_z
        )
    else:
        vertex_count, face_count = write_voxel_obj(
            mesh_path, voxels, args.voxel_size
        )

    voxel_array = np.asarray(tuple(voxels), dtype=np.int32)
    if trees and forest_bounds is not None:
        min_bound = np.asarray(
            (forest_bounds[0], forest_bounds[2], args.ground_z),
            dtype=np.float64,
        )
        max_bound = np.asarray(
            (
                forest_bounds[1],
                forest_bounds[3],
                max(tree["height"] for tree in trees),
            ),
            dtype=np.float64,
        )
    elif trees:
        min_bound = np.asarray(
            (
                min(tree["x"] - tree["crown_radius"] for tree in trees),
                min(tree["y"] - tree["crown_radius"] for tree in trees),
                args.ground_z,
            ),
            dtype=np.float64,
        )
        max_bound = np.asarray(
            (
                max(tree["x"] + tree["crown_radius"] for tree in trees),
                max(tree["y"] + tree["crown_radius"] for tree in trees),
                max(tree["height"] for tree in trees),
            ),
            dtype=np.float64,
        )
    else:
        min_bound = (
            voxel_array.min(axis=0).astype(np.float64) * args.voxel_size
        )
        max_bound = (
            voxel_array.max(axis=0).astype(np.float64) + 1.0
        ) * args.voxel_size
    write_world(world_path, mesh_path, args, (min_bound, max_bound))
    write_trajectory_csv(args.output_dir / "recorded_trajectory.csv", odometry)
    if trees:
        (args.output_dir / "trees.json").write_text(
            json.dumps(trees, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    metadata = {
        "source_bag": args.bag.name,
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
        "geometry_mode": args.geometry_mode,
        "tree_grid_size": args.tree_grid_size,
        "tree_smoothing_radius": args.tree_smoothing_radius,
        "tree_min_spacing": args.tree_min_spacing,
        "tree_density_quantile": args.tree_density_quantile,
        "tree_trunk_radius": args.tree_trunk_radius,
        "tree_crown_radius": args.tree_crown_radius,
        "tree_min_height": args.tree_min_height,
        "tree_max_height": args.tree_max_height,
        "tree_count": len(trees),
        "forest_bounds": (
            {
                "min_x": forest_bounds[0],
                "max_x": forest_bounds[1],
                "min_y": forest_bounds[2],
                "max_y": forest_bounds[3],
            }
            if forest_bounds is not None
            else None
        ),
        "forest_fill_spacing": args.forest_fill_spacing,
        "forest_path_clearance": args.forest_path_clearance,
        "forest_corner_clearance": args.forest_corner_clearance,
        "forest_seed": args.forest_seed,
        "forest_statistics": forest_stats,
        "wind_speed": args.wind_speed,
        "wind_direction": normalized_direction(
            args.wind_direction
        ).tolist(),
        "world_file": world_path.name,
        "mesh_file": "meshes/{}".format(mesh_path.name),
        "mesh_vertices": vertex_count,
        "mesh_faces": face_count,
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
        "  retained voxels/trees/vertices/faces: {}/{}/{}/{}".format(
            len(voxels), len(trees), vertex_count, face_count
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
