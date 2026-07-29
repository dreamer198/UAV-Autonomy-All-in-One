"""Clock selection and rate measurement for planner message streams."""

from __future__ import annotations

import math


def rate_sample_time(runtime_mode, receipt_monotonic, message_stamp):
    """Return the clock used to measure a ROS stream's logical frequency.

    Simulation publishers are driven by /clock.  Measuring their frequency
    against host wall time would report a false rate drop whenever Gazebo's
    real-time factor falls below one.  Real-flight streams use the monotonic
    receipt clock so a forged or irregular message stamp cannot hide a slow
    publisher.
    """

    if runtime_mode not in {"simulation", "real"}:
        raise ValueError("runtime_mode must be simulation or real")
    if not math.isfinite(receipt_monotonic):
        raise ValueError("receipt_monotonic must be finite")
    if runtime_mode == "real":
        return receipt_monotonic
    if not math.isfinite(message_stamp) or message_stamp <= 0.0:
        raise ValueError("simulation message_stamp must be finite and positive")
    return message_stamp


def observed_rate(receipts, now, window_sec):
    """Measure the rate over the trailing window, or return None while warming."""

    if (
        not math.isfinite(now)
        or not math.isfinite(window_sec)
        or window_sec <= 0.0
    ):
        raise ValueError("rate clock and window must be finite and valid")
    if not receipts or now - receipts[0] < window_sec:
        return None
    cutoff = now - window_sec
    recent = [stamp for stamp in receipts if stamp >= cutoff]
    if len(recent) < 2:
        return 0.0
    elapsed = recent[-1] - recent[0]
    if elapsed <= 0.0:
        return 0.0
    return float(len(recent) - 1) / elapsed
