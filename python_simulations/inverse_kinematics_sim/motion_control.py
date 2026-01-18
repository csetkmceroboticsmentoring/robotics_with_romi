"""
Motion control logic for waypoint following.
Ports logic from inverse_kinematics_sim_app/motion_control_thread.cpp
"""

import numpy as np
from typing import List, Tuple

# Add parent directory to path
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import robot models
from robot_models import (
    PidControl, 
    WheelWithEncoder,
    encoder_ticks_to_distance,
    distance_to_encoder_ticks,
    angle_to_encoder_ticks,
    encoder_ticks_to_angle
)


def heading_diff(heading: float, dest_dir: np.ndarray) -> float:
    """
    Calculate signed angle difference between current heading and destination direction.
    
    Args:
        heading: Current heading in radians
        dest_dir: Direction vector to destination
    
    Returns:
        Signed angle difference in radians
    """
    if dest_dir[0] == 0.0 and dest_dir[1] == 0.0:
        return 0.0
    
    head_dir = np.array([np.cos(heading), np.sin(heading)])
    cross_prod = head_dir[0] * dest_dir[1] - dest_dir[0] * head_dir[1]
    
    # Clamp dot product to avoid numerical errors with acos
    dot_prod = np.dot(head_dir, dest_dir) / np.linalg.norm(dest_dir)
    dot_prod = np.clip(dot_prod, -1.0, 1.0)
    
    angle = np.arccos(dot_prod)
    return -angle if cross_prod < 0.0 else angle


def clamp(val: float, min_range: float, max_range: float) -> float:
    """Clamp value to range."""
    return min(max(min_range, val), max_range)


class WayPoint:
    """Waypoint with position and yaw."""
    
    def __init__(self, x: float, y: float, yaw: float = 0.0):
        self.pos = np.array([x, y])
        self.yaw = yaw


class MotionController:
    """
    Motion controller for waypoint following.
    Uses PID controllers for heading and distance control.
    """
    
    def __init__(self):
        # Robot state
        self.heading = 0.0
        self.pos = np.array([0.0, 0.0])
        
        # Waypoint queue
        self.waypoints = []
        self.current_dest = WayPoint(0.0, 0.0, 0.0)
        
        # PID controllers (from C++ version)
        self.heading_pid = PidControl(kp=0.125, ki=0.0, kd=0.01)
        self.distance_pid = PidControl(kp=1.5, ki=0.0, kd=0.01)
        
        # Wheel simulators (from C++ version)
        self.left_wheel = WheelWithEncoder(kp=0.1613, ki=12.9032, kd=0.0)
        self.right_wheel = WheelWithEncoder(kp=0.1613, ki=12.9032, kd=0.0)
        
        # Set PID targets to 0.0
        self.heading_pid.set_target(0.0)
        self.distance_pid.set_target(0.0)
        
        # Trajectory history
        self.trajectory = []
        
    def add_waypoint(self, x: float, y: float):
        """Add a waypoint to the queue."""
        self.waypoints.append(WayPoint(x, y, 0.0))
        
    def update(self, dt: float = 0.025) -> Tuple[float, float, float]:
        """
        Update motion control (called from timeout).
        
        Args:
            dt: Time step in seconds
        
        Returns:
            Tuple of (x, y, heading)
        """
        # Calculate distance and heading error to destination
        dest_vector = self.current_dest.pos - self.pos
        d = np.linalg.norm(dest_vector)
        hdiff = heading_diff(self.heading, dest_vector)
        
        # Reached destination, get next waypoint
        if d < 0.02:
            if self.waypoints:
                self.current_dest = self.waypoints.pop(0)
                print(f"New dest: {self.current_dest.pos[0]:.3f}, {self.current_dest.pos[1]:.3f}")
            # If no waypoints, stay at current position
            return self.pos[0], self.pos[1], self.heading
        
        # Compute wheel velocities using PID control
        # PID controllers have target set to 0.0, so we pass negative error to get positive correction
        dist_corr = int(self.distance_pid.process(-distance_to_encoder_ticks(d), dt, -50.0, 50.0))
        correction = int(self.heading_pid.process(-angle_to_encoder_ticks(hdiff), dt, -50.0, 50.0))
        
        # Set wheel targets with clamping
        right_target = int(clamp(dist_corr * abs(np.cos(hdiff)) + correction, -50, 50))
        left_target = int(clamp(dist_corr * abs(np.cos(hdiff)) - correction, -50, 50))
        
        self.right_wheel.set_target(right_target)
        self.left_wheel.set_target(left_target)
        
        # Update odometry from wheel encoder velocities
        lv = self.left_wheel.update()
        rv = self.right_wheel.update()
        cv = (lv + rv) / 2.0
        
        self.heading += encoder_ticks_to_angle(rv - lv)
        dist = encoder_ticks_to_distance(cv)
        self.pos += np.array([dist * np.cos(self.heading), dist * np.sin(self.heading)])
        
        # Record trajectory
        if len(self.trajectory) == 0 or np.linalg.norm(self.pos - np.array(self.trajectory[-1])) > 0.01:
            self.trajectory.append(self.pos.tolist())
            if len(self.trajectory) > 1000:
                self.trajectory.pop(0)
        
        return self.pos[0], self.pos[1], self.heading
    
    def reset(self):
        """Reset motion controller state."""
        self.heading = 0.0
        self.pos = np.array([0.0, 0.0])
        self.waypoints.clear()
        self.current_dest = WayPoint(0.0, 0.0, 0.0)
        self.trajectory.clear()
        
        # Reset PID controllers
        self.heading_pid.prev_err = 0.0
        self.heading_pid.err_integral = 0.0
        self.distance_pid.prev_err = 0.0
        self.distance_pid.err_integral = 0.0
        
        # Reset wheels
        self.left_wheel.reset()
        self.right_wheel.reset()

