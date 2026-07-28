"""ROS-independent helpers for planner-neutral visualization."""

import math

import numpy as np


_NUMPY_FORMATS = {
    "f": "f4",
    "d": "f8",
}


def filter_point_records(
    data,
    width,
    height,
    point_step,
    row_step,
    coordinate_specs,
    is_bigendian,
    min_z,
    max_z,
):
    """Keep finite XYZ records whose z coordinate lies in the display band.

    The complete point record is copied, so optional fields such as intensity
    remain intact without making the common layer understand planner-specific
    point semantics.
    """

    width = int(width)
    height = int(height)
    point_step = int(point_step)
    row_step = int(row_step)
    min_z = float(min_z)
    max_z = float(max_z)
    if width < 0 or height < 0 or point_step <= 0 or row_step < 0:
        raise ValueError("point-cloud dimensions and strides are invalid")
    if row_step < width * point_step:
        raise ValueError("point-cloud row_step is smaller than its points")
    if not math.isfinite(min_z) or not math.isfinite(max_z) or max_z <= min_z:
        raise ValueError("visualization z limits must be finite and increasing")
    if set(coordinate_specs) != {"x", "y", "z"}:
        raise ValueError("coordinate_specs must contain exactly x, y, and z")

    endian = ">" if bool(is_bigendian) else "<"
    for name, raw_spec in coordinate_specs.items():
        if len(raw_spec) != 2:
            raise ValueError("coordinate field spec must contain offset and format")

    try:
        raw = memoryview(data)
    except TypeError:
        raw = memoryview(bytes(data))
    if not raw.contiguous or raw.itemsize != 1:
        raw = memoryview(bytes(raw))
    expected_size = row_step * height
    if len(raw) < expected_size:
        raise ValueError("point-cloud data is shorter than row_step * height")

    names = ("x", "y", "z")
    offsets = []
    formats = []
    for name in names:
        offset, field_format = coordinate_specs[name]
        offset = int(offset)
        field_format = str(field_format)
        if field_format not in _NUMPY_FORMATS:
            raise ValueError("{} has an unsupported float format".format(name))
        dtype = np.dtype(endian + _NUMPY_FORMATS[field_format])
        if offset < 0 or offset + dtype.itemsize > point_step:
            raise ValueError("{} field lies outside point_step".format(name))
        offsets.append(offset)
        formats.append(dtype)

    coordinate_dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": point_step,
        }
    )
    coordinates = np.ndarray(
        shape=(height, width),
        dtype=coordinate_dtype,
        buffer=raw,
        strides=(row_step, point_step),
    )
    keep = (
        np.isfinite(coordinates["x"])
        & np.isfinite(coordinates["y"])
        & np.isfinite(coordinates["z"])
        & (coordinates["z"] >= min_z)
        & (coordinates["z"] <= max_z)
    )
    kept = int(np.count_nonzero(keep))
    records = np.ndarray(
        shape=(height, width),
        dtype=np.dtype(("V", point_step)),
        buffer=raw,
        strides=(row_step, point_step),
    )
    return records[keep].tobytes(order="C"), kept


def fixed_bounds_edges(minimum, maximum):
    """Return the 24 endpoints of the 12 edges of a finite 3-D AABB."""

    minimum = tuple(float(value) for value in minimum)
    maximum = tuple(float(value) for value in maximum)
    if (
        len(minimum) != 3
        or len(maximum) != 3
        or not all(math.isfinite(value) for value in minimum + maximum)
        or any(low >= high for low, high in zip(minimum, maximum))
    ):
        raise ValueError("fixed map bounds must be finite and increasing")

    x0, y0, z0 = minimum
    x1, y1, z1 = maximum
    corners = (
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    )
    edge_indices = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    )
    return tuple(corners[index] for edge in edge_indices for index in edge)
