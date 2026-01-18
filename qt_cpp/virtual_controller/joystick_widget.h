#pragma once

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>

class JoystickWidget : public QWidget {
  Q_OBJECT

public:
  enum Orientation {
    Horizontal,
    Vertical
  };

  explicit JoystickWidget(Orientation orientation, 
                          double min, 
                          double max, 
                          double neutral = 0.0,
                          QWidget* parent = nullptr);
  
  double minimum() const { return min_value; }
  double maximum() const { return max_value; }
  double value() const { return current_value; }

signals:
  void valueChanged(double value);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  bool event(QEvent* event) override;

private:
  void setValue(double value);
  void updateValueFromMouse(const QPoint& pos);
  double normalizedToValue(double normalized) const;
  double valueToNormalized(double value) const;
  QPointF getHandleCenter() const;

  Orientation orientation;
  double min_value;
  double max_value;
  double current_value;
  double neutral_value;
  bool is_dragging;
  int active_touch_id;  // Track which touch point is controlling this joystick
};

