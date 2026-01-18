#!/usr/bin/env python3
"""Test script for the virtual controller widgets"""

import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from PySide6.QtCore import Qt

from joystick_widget import JoystickWidget
from control_widget import ControlWidget


class TestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Virtual Controller Test")
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Create control widget
        self.control_widget = ControlWidget()
        layout.addWidget(self.control_widget)
        
        # Create labels to display values
        self.left_label = QLabel("Left: 0.00")
        self.left_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.left_label)
        
        self.right_label = QLabel("Right: 0.00")
        self.right_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.right_label)
        
        # Connect signals
        self.control_widget.valuesChanged.connect(self.on_values_changed)
        
        self.resize(900, 500)
    
    def on_values_changed(self, left, right):
        self.left_label.setText(f"Left (Forward): {left:.2f}")
        self.right_label.setText(f"Right (Turn): {right:.2f}")
        
        # Calculate differential drive velocities
        k1 = 1.0
        k2 = 0.5
        left_vel = k1 * left - k2 * right
        right_vel = k1 * left + k2 * right
        
        print(f"Left: {left:.2f}, Right: {right:.2f} -> "
              f"Wheel L: {left_vel:.2f}, Wheel R: {right_vel:.2f}")


def main():
    app = QApplication(sys.argv)
    window = TestWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()

