"""
Wheel with encoder simulation.
Simulates a motor with PID velocity control and encoder feedback.
Ported from robot_sim_models/wheel_with_encoder.h
"""

import numpy as np
from .pid_control import PidControl


class WheelWithEncoder:
    """
    Simulates a wheel with encoder, matching the C++ version.
    Uses PID velocity controller with first-order motor dynamics.
    Velocity is in encoder ticks per sample (25ms period).
    """
    
    def __init__(self, kp: float, ki: float, kd: float):
        """
        Initialize wheel with PID parameters.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.pid = PidControl(kp, ki, kd)
        
        # Motor state (velocity in encoder ticks per sample)
        self.prev_pwm_1 = 0.0
        self.prev_vel_1 = 0.0
        self.prev_vel_2 = 0.0
        
    def set_target(self, target: int):
        """
        Set target velocity in encoder ticks per sample.
        
        Args:
            target: Target velocity in encoder ticks per sample
        """
        self.pid.set_target(float(target))
    
    def update(self) -> int:
        """
        Update wheel simulation (PID control, motor dynamics, noise).
        Called every timestep to advance the simulation.
        
        Returns:
            Current velocity in encoder ticks per sample (with noise)
        """
        # PID control output (PWM command)
        pwm = self.pid.process(self.prev_vel_1, 0.025, -250.0, 250.0)
        
        # First-order motor dynamics model
        # Difference equation: v[n] = 0.95*v[n-1] - 0.239*v[n-2] + 0.1232*pwm[n-1]
        curr_vel = 0.95 * self.prev_vel_1 - 0.239 * self.prev_vel_2 + 0.1232 * self.prev_pwm_1
        
        self.prev_pwm_1 = pwm
        self.prev_vel_2 = self.prev_vel_1
        
        # Add Gaussian noise to simulate encoder noise
        # Only add noise when motor is running
        noise_std = 0.0 if self.pid.get_target() == 0.0 else 2.0
        curr_vel_with_noise = np.random.normal(curr_vel, noise_std)
        
        self.prev_vel_1 = np.ceil(curr_vel_with_noise)
        return int(self.prev_vel_1)
    
    def reset(self):
        """Reset wheel state to zero."""
        self.prev_pwm_1 = 0.0
        self.prev_vel_1 = 0.0
        self.prev_vel_2 = 0.0
        self.pid.set_target(0.0)
        self.pid.prev_err = 0.0
        self.pid.err_integral = 0.0

