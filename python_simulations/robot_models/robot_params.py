"""
Robot physical parameters and conversion functions.
"""

import numpy as np


# Robot physical parameters
WHEEL_DIAMETER = 0.07  # meters
DISTANCE_BETWEEN_WHEELS = 0.14  # meters
ENCODER_TICKS_PER_ROTATION = 1440


def encoder_ticks_to_distance(enc_ticks: float) -> float:
    """Convert encoder ticks to distance in meters."""
    return (WHEEL_DIAMETER * np.pi / ENCODER_TICKS_PER_ROTATION) * enc_ticks


def distance_to_encoder_ticks(dist: float) -> float:
    """Convert distance in meters to encoder ticks."""
    return np.round((ENCODER_TICKS_PER_ROTATION * dist) / (WHEEL_DIAMETER * np.pi))


def angle_to_encoder_ticks(theta_rad: float) -> float:
    """Convert angle in radians to encoder ticks."""
    return distance_to_encoder_ticks(DISTANCE_BETWEEN_WHEELS * theta_rad)


def encoder_ticks_to_angle(enc_ticks: float) -> float:
    """Convert encoder ticks to angle in radians."""
    return encoder_ticks_to_distance(enc_ticks) / DISTANCE_BETWEEN_WHEELS

