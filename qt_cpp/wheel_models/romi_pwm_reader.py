#!/usr/bin/env python3
"""
Python program to read and analyze Romi PWM and encoder data from CSV file.
"""

import csv
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
from sysidentpy.model_structure_selection import FROLS
from sysidentpy.basis_function._basis_function import Polynomial
from sysidentpy.metrics import mean_squared_error


def read_romi_data(file_path):
    """
    Read Romi PWM and encoder data from CSV file.
    
    Args:
        file_path (str): Path to the CSV file
        
    Returns:
        list: List of dictionaries with column data
    """
    data = []
    
    try:
        with open(file_path, "r") as file:
            csv_reader = csv.reader(file)
            
            for row_num, row in enumerate(csv_reader, 1):
                if len(row) >= 4:  # Ensure we have at least 4 columns
                    try:
                        data_point = {
                            "row": row_num,
                            "left_pwm": int(row[0]),
                            "right_pwm": int(row[1]),
                            "left_enc_ticks": int(row[2]),
                            "right_enc_ticks": int(row[3])
                        }
                        data.append(data_point)
                    except ValueError:
                        print(f"Warning: Skipping row {row_num} - invalid data: {row}")
                        continue
                else:
                    print(f"Warning: Skipping row {row_num} - insufficient columns: {row}")
        
        print(f"Successfully read {len(data)} data points from {file_path}")
        return data
        
    except FileNotFoundError:
        print(f"Error: File \"{file_path}\" not found.")
        return None
    except Exception as e:
        print(f"Error reading file: {e}")
        return None


def plot_data(data):
    """
    Create plots to visualize the PWM and encoder data.
    
    Args:
        data (list): List of dictionaries containing the data
    """
    if not data:
        return
        
    # Extract data for plotting
    rows = [d["row"] for d in data]
    left_pwm_values = [d["left_pwm"] for d in data]
    left_enc_values = [d["left_enc_ticks"] for d in data]
    right_pwm_values = [d["right_pwm"] for d in data]
    right_enc_values = [d["right_enc_ticks"] for d in data]
    
    # Create figure with subplots
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle('Romi PWM and Encoder Data Analysis', fontsize=16)
    
    # Plot 1: Left PWM and Left Encoder together
    ax1 = axes[0]
    
    # Plot both PWM and Encoder on same y-axis
    line1 = ax1.plot(rows, left_pwm_values, 'b-', label='Left PWM', linewidth=1.5, alpha=0.8)
    line2 = ax1.plot(rows, left_enc_values, 'g-', label='Left Encoder', linewidth=1.5, alpha=0.8)
    
    ax1.set_xlabel('Data Point Index')
    ax1.set_ylabel('Value')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left')
    ax1.set_title('Left PWM and Left Encoder Ticks')
    
    # Plot 2: Right PWM and Right Encoder together
    ax2 = axes[1]
    
    # Plot both PWM and Encoder on same y-axis
    line3 = ax2.plot(rows, right_pwm_values, 'r-', label='Right PWM', linewidth=1.5, alpha=0.8)
    line4 = ax2.plot(rows, right_enc_values, 'orange', label='Right Encoder', linewidth=1.5, alpha=0.8)
    
    ax2.set_xlabel('Data Point Index')
    ax2.set_ylabel('Value')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left')
    ax2.set_title('Right PWM and Right Encoder Ticks')
    
    plt.tight_layout()
    plt.show()

def decode_term(lag):
    item = int(lag/1000)
    index = int(lag%1000)
    letter = 'y' if item == 1 else 'x'
    term_str = f"{letter}[k-{index}]"
    return term_str

def readable_equation(model):
    """
    Print the model equation in a readable format.
    
    Args:
        model: FROLS model object
    """
    if not hasattr(model, 'final_model') or not hasattr(model, 'theta'):
        print("Model does not have final_model or theta attributes")
        return
    
    if model.final_model is None or model.theta is None:
        print("Model structure or parameters are None")
        return
    
    equation_terms = []
    
    for i, (term, coeff) in enumerate(zip(model.final_model, model.theta)):
        if abs(coeff) < 1e-10:  # Skip very small coefficients
            continue
            
        term_str = ""
        # Parse the term structure
        for item in term:
            lag = int(item)  # Input lag
            
            if term_str == "":
                term_str = decode_term(lag)
            else:
                term_str = term_str + "*" + decode_term(lag)
        
        equation_terms.append(f"{coeff[0]}*{term_str}")
    
    str_equation = "y[k] = " + " + ".join(equation_terms).replace("+ -", "- ")
    return str_equation


def fit_second_order_model(pwm_values, enc_values): 
    # Prepare data for sysidentpy (input-output format)
    # Convert to proper format: u=input, y=output
    u = pwm_values.reshape(-1, 1)  # Input: left_pwm
    y = enc_values.reshape(-1, 1)  # Output: left_enc_ticks
    
    # Define polynomial basis function for second-order model
    basis_function = Polynomial(degree=1)
    
    # Create FROLS model for second-order identification
    model = FROLS(
        order_selection=True,
        n_info_values=5,
        extended_least_squares=False,
        ylag=2,  # Output lag
        xlag=1,  # Input lag
        info_criteria='aic',
        estimator='least_squares',
        basis_function=basis_function
    )
    
    # Fit the model
    model.fit(X=u, y=y)

    return model

def main():
    """Main function to run the Romi data analysis."""
    
    # Check command line arguments
    if len(sys.argv) != 2:
        print("Usage: python romi_pwm_reader.py <file_path>")
        print("Example: python romi_pwm_reader.py romi_pwm_enc_data_25ms_sampling.txt")
        sys.exit(1)
    
    # Get file path from command line argument
    file_path = sys.argv[1]
    
    print("Romi PWM and Encoder Data Reader")
    print(f"Reading data from: {file_path}")
    
    # Read the data
    data = read_romi_data(file_path)
    
    if data:
        # Extract data for identification
        left_pwm_values = np.array([d["left_pwm"] for d in data])
        left_enc_values = np.array([d["left_enc_ticks"] for d in data])
        # Fit second-order model
        left_wheel_model = fit_second_order_model(left_pwm_values, left_enc_values)

        # Extract data for identification
        right_pwm_values = np.array([d["right_pwm"] for d in data])
        right_enc_values = np.array([d["right_enc_ticks"] for d in data])
        # Fit second-order model
        right_wheel_model = fit_second_order_model(right_pwm_values, right_enc_values)
        
        print("=" * 100)
        print(f"Left wheel:\n{readable_equation(left_wheel_model)}")
        print("\n")
        print(f"Right wheel:\n{readable_equation(right_wheel_model)}")
        print("\ny[k] is the velocity as `encoder ticks` and x[k] the PWM value")
        print("=" * 100)

        #plot_data(data)

if __name__ == "__main__":
    main()
