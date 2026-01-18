"""
Common robot models and utilities.
Contains PID control, wheel encoder simulation, and robot parameters.
"""

from .pid_control import PidControl
from .wheel_with_encoder import WheelWithEncoder
from .robot_params import (
    WHEEL_DIAMETER,
    DISTANCE_BETWEEN_WHEELS,
    ENCODER_TICKS_PER_ROTATION,
    encoder_ticks_to_distance,
    distance_to_encoder_ticks,
    angle_to_encoder_ticks,
    encoder_ticks_to_angle
)

__all__ = [
    'PidControl',
    'WheelWithEncoder',
    'WHEEL_DIAMETER',
    'DISTANCE_BETWEEN_WHEELS',
    'ENCODER_TICKS_PER_ROTATION',
    'encoder_ticks_to_distance',
    'distance_to_encoder_ticks',
    'angle_to_encoder_ticks',
    'encoder_ticks_to_angle'
]

