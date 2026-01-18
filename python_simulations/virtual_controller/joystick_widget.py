"""
Joystick widget for virtual controller.
"""

from PySide6.QtCore import Qt, QPointF, QRectF, QEvent, Signal
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QRadialGradient


class JoystickWidget(QWidget):
    """
    A customizable joystick widget supporting horizontal and vertical orientations.
    Supports both mouse and touch input with auto-return to neutral position.
    """
    
    # Orientation enum
    Horizontal = 0
    Vertical = 1
    
    valueChanged = Signal(float)
    
    def __init__(self, orientation, min_val, max_val, neutral=0.0, parent=None):
        super().__init__(parent)
        
        self.orientation = orientation
        self.min_value = min_val
        self.max_value = max_val
        self.current_value = self._clamp(neutral, min_val, max_val)
        self.neutral_value = self._clamp(neutral, min_val, max_val)
        self.is_dragging = False
        self.active_touch_id = -1  # -1 means no active touch
        
        self.setMinimumSize(40, 150)
        self.setMouseTracking(False)
        
        # Enable touch events
        self.setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents, True)
    
    @staticmethod
    def _clamp(value, low, high):
        """Helper function to clamp value between low and high"""
        return max(low, min(high, value))
    
    def minimum(self):
        return self.min_value
    
    def maximum(self):
        return self.max_value
    
    def value(self):
        return self.current_value
    
    def _set_value(self, value):
        """Internal method to set value"""
        clamped = self._clamp(value, self.min_value, self.max_value)
        if clamped != self.current_value:
            self.current_value = clamped
            self.update()
            self.valueChanged.emit(self.current_value)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Handle dimensions
        handle_radius = 30
        margin = handle_radius + 5  # Extra padding to prevent cutoff
        
        # Draw background track
        if self.orientation == self.Vertical:
            track_width = 16
            track_rect = ((self.width() - track_width) // 2, margin,
                         track_width, self.height() - 2 * margin)
        else:
            track_height = 16
            track_rect = (margin, (self.height() - track_height) // 2,
                         self.width() - 2 * margin, track_height)
        
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(80, 80, 80))
        painter.drawRoundedRect(*track_rect, 4, 4)
        
        # Draw handle
        handle_center = self._get_handle_center()
        
        # Handle shadow
        painter.setBrush(QColor(0, 0, 0, 50))
        painter.drawEllipse(handle_center + QPointF(2, 2), 30, 30)
        
        # Handle
        gradient = QRadialGradient(handle_center, 30)
        if self.is_dragging:
            gradient.setColorAt(0, QColor(100, 150, 255))
            gradient.setColorAt(1, QColor(50, 100, 200))
        else:
            gradient.setColorAt(0, QColor(200, 200, 200))
            gradient.setColorAt(1, QColor(150, 150, 150))
        
        painter.setBrush(gradient)
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawEllipse(handle_center, 30, 30)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_dragging = True
            self._update_value_from_mouse(event.pos())
            self.update()
    
    def mouseMoveEvent(self, event):
        if self.is_dragging:
            self._update_value_from_mouse(event.pos())
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_dragging = False
            # Return to neutral position on release
            self._set_value(self.neutral_value)
            self.update()
    
    def event(self, event):
        if event.type() in (QEvent.Type.TouchBegin, QEvent.Type.TouchUpdate, QEvent.Type.TouchEnd):
            touch_points = event.touchPoints()
            
            for touch_point in touch_points:
                pos = touch_point.pos()
                widget_rect = QRectF(self.rect())
                
                # Check if touch is within this widget
                if not widget_rect.contains(pos):
                    continue
                
                touch_id = touch_point.id()
                
                if event.type() == QEvent.Type.TouchBegin:
                    # New touch - claim it if we don't have an active touch
                    if self.active_touch_id == -1:
                        self.active_touch_id = touch_id
                        self.is_dragging = True
                        self._update_value_from_mouse(pos.toPoint())
                elif event.type() == QEvent.Type.TouchUpdate:
                    # Update - only process if this is our active touch
                    if self.active_touch_id == touch_id:
                        self._update_value_from_mouse(pos.toPoint())
                elif event.type() == QEvent.Type.TouchEnd:
                    # Touch ended - release if this was our active touch
                    if self.active_touch_id == touch_id:
                        self.active_touch_id = -1
                        self.is_dragging = False
                        self._set_value(self.neutral_value)
                        self.update()
            
            event.accept()
            return True
        
        return super().event(event)
    
    def _update_value_from_mouse(self, pos):
        handle_radius = 30
        margin = handle_radius + 5
        
        if self.orientation == self.Vertical:
            track_start = margin
            track_length = self.height() - 2 * margin
            y = self._clamp(pos.y(), track_start, track_start + track_length)
            # Invert: top = max, bottom = min
            normalized = 1.0 - (y - track_start) / track_length
        else:
            track_start = margin
            track_length = self.width() - 2 * margin
            x = self._clamp(pos.x(), track_start, track_start + track_length)
            normalized = (x - track_start) / track_length
        
        self._set_value(self._normalized_to_value(normalized))
    
    def _normalized_to_value(self, normalized):
        return self.min_value + normalized * (self.max_value - self.min_value)
    
    def _value_to_normalized(self, value):
        if self.max_value == self.min_value:
            return 0.0
        return (value - self.min_value) / (self.max_value - self.min_value)
    
    def _get_handle_center(self):
        normalized = self._value_to_normalized(self.current_value)
        
        handle_radius = 30
        margin = handle_radius + 5
        
        if self.orientation == self.Vertical:
            track_start = margin
            track_length = self.height() - 2 * margin
            # Invert: top = max, bottom = min
            y = track_start + (1.0 - normalized) * track_length
            return QPointF(self.width() / 2.0, y)
        else:
            track_start = margin
            track_length = self.width() - 2 * margin
            x = track_start + normalized * track_length
            return QPointF(x, self.height() / 2.0)

