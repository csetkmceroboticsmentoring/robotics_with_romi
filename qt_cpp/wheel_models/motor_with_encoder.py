import random

class MotorWithEncoder:
    """
    Motor with encoder simulation using difference equation model.
    
    Implements: y[n] = b0*x[n-1] + a0*y[n-1] + a1*y[n-2]
    
    Where:
        - y[n] is current velocity output
        - x[n-1] is previous PWM input
        - y[n-1] is previous velocity output
        - y[n-2] is velocity output from two steps ago
    """
    
    def __init__(self, b0, a0, a1):
        """
        Initialize motor with difference equation coefficients.
        
        Args:
            b0 (float): Input coefficient (PWM to velocity gain)
            a0 (float): Previous output coefficient (velocity feedback)
            a1 (float): Two-steps-back output coefficient (velocity history)
        """
        # Store difference equation coefficients
        self.b0 = b0
        self.a0 = a0
        self.a1 = a1
        
        # Initialize state history
        self.prev_pwm_input = 0.0    # x[n-1] - previous PWM input
        self.prev_velocity_1 = 0.0   # y[n-1] - previous velocity output
        self.prev_velocity_2 = 0.0   # y[n-2] - velocity from two steps ago
        
    def update(self, pwm_input):
        """
        Update motor dynamics using difference equation.
        
        Args:
            pwm_input (float): PWM input (-255 to 255)
            
        Returns:
            float: Current velocity output (encoder ticks/sec)
        """
        # Calculate current velocity: y[n] = b0*x[n-1] + a0*y[n-1] + a1*y[n-2]
        current_velocity = (self.b0 * self.prev_pwm_input + 
                           self.a0 * self.prev_velocity_1 + 
                           self.a1 * self.prev_velocity_2)
        
        # Add noise to simulate encoder measurement noise (normal distribution, 1.5% of current velocity)
        #noise_std = abs(current_velocity) * 0.015  # 1.5% of current velocity
        #noise = random.gauss(0, noise_std)  # Mean=0, StdDev=3% of velocity
        #current_velocity += noise
        
        # Update state history for next iteration
        self.prev_pwm_input = pwm_input      # Store current PWM for next step
        self.prev_velocity_2 = self.prev_velocity_1  # Shift: y[n-2] = y[n-1]
        self.prev_velocity_1 = current_velocity      # Shift: y[n-1] = y[n]
        
        return current_velocity
    
    def reset(self):
        """Reset motor to initial state (all zeros)."""
        self.prev_pwm_input = 0.0
        self.prev_velocity_1 = 0.0
        self.prev_velocity_2 = 0.0
    


