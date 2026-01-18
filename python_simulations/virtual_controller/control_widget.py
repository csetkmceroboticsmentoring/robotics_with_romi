"""
Dual-joystick control widget for differential drive robots.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QWidget, QHBoxLayout

from joystick_widget import JoystickWidget


class ControlWidget(QWidget):
    """
    Control widget with two joysticks for differential drive control.
    Left joystick: vertical, forward speed (0.0 to 1.0)
    Right joystick: horizontal, turn rate (-1.0 to +1.0)
    """
    
    leftValueChanged = Signal(float)
    rightValueChanged = Signal(float)
    valuesChanged = Signal(float, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Enable touch events for the control widget
        self.setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents, True)
        
        # Create main layout
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(20)
        
        # Left joystick (vertical, range 0.0 to 1.0)
        self.left_joystick = JoystickWidget(JoystickWidget.Vertical, 0.0, 1.0, 0.0, self)
        self.left_joystick.setFixedSize(400, 350)
        
        # Right joystick (horizontal, range -1.0 to +1.0)
        self.right_joystick = JoystickWidget(JoystickWidget.Horizontal, -1.0, 1.0, 0.0, self)
        self.right_joystick.setFixedSize(400, 350)
        
        # Add joysticks directly to main layout
        main_layout.addWidget(self.left_joystick, 0, Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.right_joystick, 0, Qt.AlignmentFlag.AlignCenter)
        
        # Connect signals
        self.left_joystick.valueChanged.connect(self._on_left_changed)
        self.right_joystick.valueChanged.connect(self._on_right_changed)
    
    def left_value(self):
        return self.left_joystick.value()
    
    def right_value(self):
        return self.right_joystick.value()
    
    def _on_left_changed(self, value):
        self.leftValueChanged.emit(value)
        self.valuesChanged.emit(self.left_joystick.value(), self.right_joystick.value())
    
    def _on_right_changed(self, value):
        self.rightValueChanged.emit(value)
        self.valuesChanged.emit(self.left_joystick.value(), self.right_joystick.value())

