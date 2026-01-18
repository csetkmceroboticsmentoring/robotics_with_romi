import matplotlib.pyplot as plt
import numpy as np

def read_romi_data(filename):
    """Read ROMI PWM and encoder data from file."""
    left_pwm = []
    right_pwm = []
    left_vel = []
    right_vel = []
    time_data = []
    
    with open(filename, 'r') as file:
        for line_num, line in enumerate(file, 1):
            line = line.strip()
            if line:  # Skip empty lines
                try:
                    # Split by comma and convert to float
                    parts = [float(x.strip()) for x in line.split(',')]
                    if len(parts) >= 4:
                        left_pwm.append(parts[0])
                        right_pwm.append(parts[1])
                        left_vel.append(parts[2])
                        right_vel.append(parts[3])
                        # Time in seconds (25ms = 0.025s per sample)
                        time_data.append(line_num * 0.025)
                except ValueError:
                    print(f"Warning: Could not parse line {line_num}: {line}")
                    continue
    
    return time_data, left_pwm, right_pwm, left_vel, right_vel

def plot_romi_data(time_data, left_pwm, right_pwm, left_vel, right_vel):
    """Plot ROMI PWM and velocity data."""
    
    # Set dark theme
    plt.style.use('dark_background')
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Left PWM
    axes[0, 0].plot(time_data, left_pwm, 'cyan', label='Left PWM', linewidth=1.5)
    axes[0, 0].set_xlabel('Time (s)', color='white')
    axes[0, 0].set_ylabel('PWM Value', color='white')
    axes[0, 0].set_title('Left Motor PWM', color='white')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3, color='gray')
    axes[0, 0].tick_params(colors='white')
    
    # Plot 2: Right PWM
    axes[0, 1].plot(time_data, right_pwm, 'orange', label='Right PWM', linewidth=1.5)
    axes[0, 1].set_xlabel('Time (s)', color='white')
    axes[0, 1].set_ylabel('PWM Value', color='white')
    axes[0, 1].set_title('Right Motor PWM', color='white')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3, color='gray')
    axes[0, 1].tick_params(colors='white')
    
    # Plot 3: Left Velocity
    axes[1, 0].plot(time_data, left_vel, 'lime', label='Left Velocity', linewidth=1.5)
    axes[1, 0].set_xlabel('Time (s)', color='white')
    axes[1, 0].set_ylabel('Velocity (Encoder ticks/sec)', color='white')
    axes[1, 0].set_title('Left Motor Velocity', color='white')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3, color='gray')
    axes[1, 0].tick_params(colors='white')
    
    # Plot 4: Right Velocity
    axes[1, 1].plot(time_data, right_vel, 'magenta', label='Right Velocity', linewidth=1.5)
    axes[1, 1].set_xlabel('Time (s)', color='white')
    axes[1, 1].set_ylabel('Velocity (Encoder ticks/sec)', color='white')
    axes[1, 1].set_title('Right Motor Velocity', color='white')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3, color='gray')
    axes[1, 1].tick_params(colors='white')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Read data from file
    filename = "romi_pwm_enc_data_25ms_sampling.txt"
    time_data, left_pwm, right_pwm, left_vel, right_vel = read_romi_data(filename)
    
    print(f"Data loaded successfully!")
    print(f"Total samples: {len(time_data)}")
    print(f"Duration: {time_data[-1]:.2f} seconds")
    print(f"Left PWM range: {min(left_pwm):.0f} to {max(left_pwm):.0f}")
    print(f"Right PWM range: {min(right_pwm):.0f} to {max(right_pwm):.0f}")
    print(f"Left velocity range: {min(left_vel):.0f} to {max(left_vel):.0f}")
    print(f"Right velocity range: {min(right_vel):.0f} to {max(right_vel):.0f}")
    
    # Plot the data
    plot_romi_data(time_data, left_pwm, right_pwm, left_vel, right_vel)
