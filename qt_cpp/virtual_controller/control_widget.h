#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "joystick_widget.h"

class ControlWidget : public QWidget {
  Q_OBJECT

public:
  explicit ControlWidget(QWidget* parent = nullptr);

  double leftValue() const { return left_joystick->value(); }
  double rightValue() const { return right_joystick->value(); }

signals:
  void leftValueChanged(double value);
  void rightValueChanged(double value);
  void valuesChanged(double left, double right);

private slots:
  void onLeftChanged(double value);
  void onRightChanged(double value);

private:
  JoystickWidget* left_joystick = nullptr;
  JoystickWidget* right_joystick = nullptr;
};

