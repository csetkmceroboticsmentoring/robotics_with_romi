import time
import math


class PIDController:
    """
    A Proportional-Integral-Derivative (PID) controller implementation.
    
    This class provides a standard PID control algorithm with configurable
    gains and output limits.
    """
    
    def __init__(self, kp, ki, kd, output_limits):
        """
        Initialize the PID controller.
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain  
            kd (float): Derivative gain
            output_limits (tuple): (min, max) output limits
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Output limits
        self.output_limits = output_limits
        
        # Internal state
        self.reset()
        
        
    def set_gains(self, kp, ki, kd):
        """Update controller gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()
            
    def reset(self):
        """Reset the controller state."""
        self._last_error = 0.0
        self._error_integral = 0.0
        
    def update(self, measured_velocity, setpoint_velocity, dt):
        """
        Update the PID controller and return the control output.
        
        Args:
            measured_velocity (float): Current process value
            setpoint_velocity (float): Target value for the controller
            dt (float): Time step in seconds
            
        Returns:
            float: Control output
        """
        
        # Calculate error
        error = setpoint_velocity - measured_velocity
        error_derivative = (error - self._last_error) / dt
        self._error_integral += error * dt
        self._last_error = error
        
        # Calculate PID output
        proportional_term = self.kp * error
        integral_term = self.ki * self._error_integral
        derivative_term = self.kd * error_derivative
        output_pwm = proportional_term + integral_term + derivative_term
        
        if output_pwm < self.output_limits[0]:
            output_pwm = self.output_limits[0]
            self._error_integral -= error * dt
        elif output_pwm > self.output_limits[1]:
            output_pwm = self.output_limits[1]
            self._error_integral -= error * dt
        return output_pwm
        
    def get_state(self):
        """Get the current controller state."""
        return {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'error_integral': self._error_integral,
            'last_error': self._last_error
        }
