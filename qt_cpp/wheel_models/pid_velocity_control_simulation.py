import matplotlib.pyplot as plt
from pid_controller import PIDController
from motor_with_encoder import MotorWithEncoder


def generate_step_inputs(setpoints):
    """Generate step input sequence with specified setpoints."""
    
    # Define step sequence
    samples_per_step = 120
    dt = 0.025  # 25ms time step
    
    # Initialize data storage
    time_stamp = []
    set_point_velocity = []
    
    time_current = 0.0
    
    print("Generating Step Input Sequence")
    print("=" * 50)
    print(f"Setpoints: {setpoints}")
    print(f"Samples per step: {samples_per_step}")
    print(f"Time step: {dt}s")
    print()
    
    # Generate input sequence
    for step_idx, setpoint in enumerate(setpoints):
        print(f"Step {step_idx + 1}: Input = {setpoint}")
        
        # Generate samples for this step
        for sample in range(samples_per_step):
            # Store data
            time_stamp.append(time_current)
            set_point_velocity.append(setpoint)
            
            time_current += dt
            
            # Print progress every 20 samples
            if sample % 20 == 0:
                print(f"  Sample {sample:3d}: Time={time_current:6.2f}s, Input={setpoint:6.2f}")
        
        print(f"  Final: Time={time_current:.2f}s, Input={setpoint:.2f}")
        print()
    
    return time_stamp, set_point_velocity


def plot_results(time_stamp, set_point_velocity, output_velocity, input_pwm, error_data, num_steps):
    """Plot setpoint velocity, output velocity, input PWM, and error data."""
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 16))
    
    # Plot 1: Setpoint velocity and Output velocity together
    axes[0].plot(time_stamp, set_point_velocity, 'b--', label='setpoint velocity', linewidth=2)
    axes[0].plot(time_stamp, output_velocity, 'r-', label='output velocity', linewidth=1.5)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('velocity (Encoder ticks per sampling time)')
    axes[0].set_title('setpoint velocity vs output velocity (Both in encoder ticks per sampling time)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Input PWM data
    axes[1].plot(time_stamp, input_pwm, 'g-', label='Input PWM', linewidth=1)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('PWM Value')
    axes[1].set_title('Input PWM Signal')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Error data
    axes[2].plot(time_stamp, error_data, 'orange', label='Error', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Error (Encoder ticks/sec)')
    axes[2].set_title('Control Error (Setpoint - Output)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # Plot 4: Zoomed view of velocities, PWM, and error (3-9 seconds)
    samples_per_step = 120
    dt = 0.025
    zoom_start_time = 3.0  # Start at 3 seconds
    zoom_end_time = 9.0    # End at 9 seconds
    zoom_start_sample = int(zoom_start_time / dt)  # Sample index for 3 seconds
    zoom_end_sample = int(zoom_end_time / dt)      # Sample index for 9 seconds
    
    axes[3].plot(time_stamp[zoom_start_sample:zoom_end_sample], set_point_velocity[zoom_start_sample:zoom_end_sample], 
                 'b--', label='Setpoint Velocity', linewidth=2, marker='o')
    axes[3].plot(time_stamp[zoom_start_sample:zoom_end_sample], output_velocity[zoom_start_sample:zoom_end_sample], 
                 'r-', label='Output Velocity', linewidth=1.5, marker='s')
    axes[3].plot(time_stamp[zoom_start_sample:zoom_end_sample], input_pwm[zoom_start_sample:zoom_end_sample], 
                 'g-', label='Input PWM', linewidth=1, marker='^')
    axes[3].plot(time_stamp[zoom_start_sample:zoom_end_sample], error_data[zoom_start_sample:zoom_end_sample], 
                 'orange', label='Error', linewidth=1, marker='d')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_ylabel('Velocity (Encoder ticks per sampling time) / PWM Value')
    axes[3].set_title(f'Zoomed View - 3-9 Seconds (Velocities, PWM & Error)')
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    
    # Add vertical lines to separate steps (for first three plots only)
    for i in range(1, num_steps):  # Number of steps
        for ax in axes[:3]:  # Only first three plots
            ax.axvline(x=i * 120 * 0.025, color='gray', linestyle=':', alpha=0.5)
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Generate step input sequence
    setpoints = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, -120,-110,-100,-90, -80, -70, -60, -50, -40, -30, -20, -10, 0]
    time_stamp, set_point_velocity = generate_step_inputs(setpoints)

    # PID Controller Parameters
    Kp = 2.0
    Ki = 22.5
    Kd = 0.0
    pwm_limits = (-250.0, 250.0)    
    # Create PID controller
    pid_controller = PIDController(Kp, Ki, Kd, pwm_limits)
    
    # Motor Model Parameters
    b0 = 0.12490082647581259
    a0 = 0.9290073015928384
    a1 = -0.22243586965022222
    # y[k] = b0*x[k-1] + a0*y[k-1] + a1*y[k-2]
    motor_with_encoder = MotorWithEncoder(b0, a0, a1)
    
    # Initialize output data
    output_velocity = []
    input_pwm = []
    error_data = []
    dt = 0.025  # 25ms time step
    
    current_velocity = 0.0
    # Process each input through PID controller
    for i, velocity_setpoint in enumerate(set_point_velocity):
        pwm_input = pid_controller.update(current_velocity, velocity_setpoint, dt)
        current_velocity = motor_with_encoder.update(pwm_input)
        
        # Calculate error
        error = velocity_setpoint - current_velocity

        output_velocity.append(current_velocity)
        input_pwm.append(pwm_input)
        error_data.append(error)
    
    # Plot results
    try:
        plot_results(time_stamp, set_point_velocity, output_velocity, input_pwm, error_data, len(setpoints))
    except ImportError:
        print("\nMatplotlib not available. Install with: pip install matplotlib")
        print("Data collected successfully but plotting skipped.")
    
    print(f"\nTest completed. Total samples: {len(time_stamp)}")
    print(f"Total time: {time_stamp[-1]:.2f} seconds")
