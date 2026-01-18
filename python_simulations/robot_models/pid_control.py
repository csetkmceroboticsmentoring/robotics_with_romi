"""
PID controller with anti-windup.
Ported from robot_sim_models/pid_control.cpp
"""


class PidControl:
    """
    PID controller with anti-windup.
    Matches the C++ implementation behavior.
    """
    
    def __init__(self, kp: float, ki: float, kd: float):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.target = 0.0
        self.prev_err = 0.0
        self.err_integral = 0.0
    
    def set_target(self, target: float):
        """Set target value."""
        self.target = target
    
    def get_target(self) -> float:
        """Get current target."""
        return self.target
    
    def process(self, obs: float, dt: float, min_range: float, max_range: float) -> float:
        """
        Process PID control.
        
        Args:
            obs: Observed value
            dt: Time step in seconds
            min_range: Minimum output value
            max_range: Maximum output value
        
        Returns:
            Control output clamped to [min_range, max_range]
        """
        err = self.target - obs
        der_err = (err - self.prev_err) / dt
        self.prev_err = err
        self.err_integral += err * dt
        
        out = self.Kp * err + self.Ki * self.err_integral + self.Kd * der_err
        
        # Anti-windup: prevent integral from growing when output is saturated
        if out < min_range:
            out = min_range
            self.err_integral -= err * dt
        elif out > max_range:
            out = max_range
            self.err_integral -= err * dt
        
        return out

