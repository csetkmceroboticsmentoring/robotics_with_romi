#include "control_widget.h"

ControlWidget::ControlWidget(QWidget* parent)
  : QWidget(parent) {
  
  // Enable touch events for the control widget
  setAttribute(Qt::WA_AcceptTouchEvents, true);
  
  // Create main layout
  QHBoxLayout* main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(10, 10, 10, 10);
  main_layout->setSpacing(20);

  // Left joystick (vertical, range 0.0 to 1.0)
  left_joystick = new JoystickWidget(JoystickWidget::Vertical, 0.0, 1.0, 0.0, this);
  left_joystick->setFixedSize(400, 350);

  // Right joystick (horizontal, range -1.0 to +1.0)
  right_joystick = new JoystickWidget(JoystickWidget::Horizontal, -1.0, 1.0, 0.0, this);
  right_joystick->setFixedSize(400, 350);

  // Add joysticks directly to main layout
  main_layout->addWidget(left_joystick, 0, Qt::AlignCenter);
  main_layout->addWidget(right_joystick, 0, Qt::AlignCenter);

  // Connect signals
  connect(left_joystick, &JoystickWidget::valueChanged, 
          this, &ControlWidget::onLeftChanged);
  connect(right_joystick, &JoystickWidget::valueChanged, 
          this, &ControlWidget::onRightChanged);
}

void ControlWidget::onLeftChanged(double value) {
  emit leftValueChanged(value);
  emit valuesChanged(left_joystick->value(), right_joystick->value());
}

void ControlWidget::onRightChanged(double value) {
  emit rightValueChanged(value);
  emit valuesChanged(left_joystick->value(), right_joystick->value());
}

