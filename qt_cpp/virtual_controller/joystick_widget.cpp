#include "joystick_widget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QTouchEvent>
#include <algorithm>

// Helper function for C++14 compatibility (std::clamp is C++17)
template<typename T>
constexpr const T& clamp(const T& value, const T& low, const T& high) {
  return (value < low) ? low : (high < value) ? high : value;
}

JoystickWidget::JoystickWidget(Orientation orientation,
                               double min,
                               double max,
                               double neutral,
                               QWidget* parent)
  : QWidget(parent),
    orientation(orientation),
    min_value(min),
    max_value(max),
    current_value(clamp(neutral, min, max)),
    neutral_value(clamp(neutral, min, max)),
    is_dragging(false),
    active_touch_id(-1) {  // -1 means no active touch
  setMinimumSize(40, 150);
  setMouseTracking(false);
  
  // Enable touch events
  setAttribute(Qt::WA_AcceptTouchEvents, true);
}

void JoystickWidget::setValue(double value) {
  const double clamped = clamp(value, min_value, max_value);
  if (clamped != current_value) {
    current_value = clamped;
    update();
    emit valueChanged(current_value);
  }
}

void JoystickWidget::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Handle dimensions
  const int handle_radius = 30;
  const int margin = handle_radius + 5;  // Extra padding to prevent cutoff

  // Draw background track
  QRect track_rect;
  if (orientation == Vertical) {
    const int track_width = 16;
    track_rect = QRect((width() - track_width) / 2, margin, track_width, height() - 2 * margin);
  } else {
    const int track_height = 16;
    track_rect = QRect(margin, (height() - track_height) / 2, width() - 2 * margin, track_height);
  }
  
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(80, 80, 80));
  painter.drawRoundedRect(track_rect, 4, 4);

  // Draw handle
  const QPointF handle_center = getHandleCenter();
  
  // Handle shadow
  painter.setBrush(QColor(0, 0, 0, 50));
  painter.drawEllipse(handle_center + QPointF(2, 2), 30, 30);
  
  // Handle
  QRadialGradient gradient(handle_center, 30);
  if (is_dragging) {
    gradient.setColorAt(0, QColor(100, 150, 255));
    gradient.setColorAt(1, QColor(50, 100, 200));
  } else {
    gradient.setColorAt(0, QColor(200, 200, 200));
    gradient.setColorAt(1, QColor(150, 150, 150));
  }
  painter.setBrush(gradient);
  painter.setPen(QPen(QColor(100, 100, 100), 2));
  painter.drawEllipse(handle_center, 30, 30);
}

void JoystickWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    is_dragging = true;
    updateValueFromMouse(event->pos());
    update();
  }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent* event) {
  if (is_dragging) {
    updateValueFromMouse(event->pos());
  }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    is_dragging = false;
    // Return to neutral position on release
    setValue(neutral_value);
    update();
  }
}

void JoystickWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
}

void JoystickWidget::updateValueFromMouse(const QPoint& pos) {
  double normalized;
  
  const int handle_radius = 30;
  const int margin = handle_radius + 5;
  
  if (orientation == Vertical) {
    const int track_start = margin;
    const int track_length = height() - 2 * margin;
    const int y = clamp(pos.y(), track_start, track_start + track_length);
    // Invert: top = max, bottom = min
    normalized = 1.0 - static_cast<double>(y - track_start) / track_length;
  } else {
    const int track_start = margin;
    const int track_length = width() - 2 * margin;
    const int x = clamp(pos.x(), track_start, track_start + track_length);
    normalized = static_cast<double>(x - track_start) / track_length;
  }
  
  setValue(normalizedToValue(normalized));
}

double JoystickWidget::normalizedToValue(double normalized) const {
  return min_value + normalized * (max_value - min_value);
}

double JoystickWidget::valueToNormalized(double value) const {
  if (max_value == min_value) {
    return 0.0;
  }
  return (value - min_value) / (max_value - min_value);
}

QPointF JoystickWidget::getHandleCenter() const {
  const double normalized = valueToNormalized(current_value);
  
  const int handle_radius = 30;
  const int margin = handle_radius + 5;
  
  if (orientation == Vertical) {
    const int track_start = margin;
    const int track_length = height() - 2 * margin;
    // Invert: top = max, bottom = min
    const double y = track_start + (1.0 - normalized) * track_length;
    return QPointF(width() / 2.0, y);
  } else {
    const int track_start = margin;
    const int track_length = width() - 2 * margin;
    const double x = track_start + normalized * track_length;
    return QPointF(x, height() / 2.0);
  }
}

bool JoystickWidget::event(QEvent* event) {
  switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd: {
      QTouchEvent* touch_event = static_cast<QTouchEvent*>(event);
      const QList<QTouchEvent::TouchPoint>& touch_points = touch_event->touchPoints();
      
      for (const QTouchEvent::TouchPoint& touch_point : touch_points) {
        const QPointF pos = touch_point.pos();
        const QRectF widget_rect = rect();
        
        // Check if touch is within this widget
        if (!widget_rect.contains(pos)) {
          continue;
        }
        
        const int touch_id = touch_point.id();
        
        if (event->type() == QEvent::TouchBegin) {
          // New touch - claim it if we don't have an active touch
          if (active_touch_id == -1) {
            active_touch_id = touch_id;
            is_dragging = true;
            updateValueFromMouse(pos.toPoint());
          }
        } else if (event->type() == QEvent::TouchUpdate) {
          // Update - only process if this is our active touch
          if (active_touch_id == touch_id) {
            updateValueFromMouse(pos.toPoint());
          }
        } else if (event->type() == QEvent::TouchEnd) {
          // Touch ended - release if this was our active touch
          if (active_touch_id == touch_id) {
            active_touch_id = -1;
            is_dragging = false;
            setValue(neutral_value);
            update();
          }
        }
      }
      
      event->accept();
      return true;
    }
    default:
      break;
  }
  
  return QWidget::event(event);
}

