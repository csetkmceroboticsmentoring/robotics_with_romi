/**
 * Test application for joystick and control widgets
 * 
 * Compile and run this to test the joystick widgets independently
 * from the main forward kinematics simulation.
 */

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QDebug>
#include "control_widget.h"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  QMainWindow window;
  window.setWindowTitle("Joystick Widget Test");

  QWidget* central = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(central);

  // Info label
  QLabel* info = new QLabel(
    "Left Joystick: Vertical, Range 0.0 to 1.0\n"
    "Right Joystick: Horizontal, Range -1.0 to +1.0"
  );
  info->setStyleSheet("padding: 10px; background-color: #f0f0f0;");
  layout->addWidget(info);

  // Control widget with two joysticks
  ControlWidget* control = new ControlWidget();
  layout->addWidget(control);

  // Status label to show current values
  QLabel* status = new QLabel("Left: 0.00 | Right: 0.00");
  status->setStyleSheet("padding: 10px; font-weight: bold;");
  layout->addWidget(status);

  // Connect to display values
  QObject::connect(control, &ControlWidget::valuesChanged,
    [status](double left, double right) {
      status->setText(QString("Left: %1 | Right: %2")
        .arg(left, 0, 'f', 2)
        .arg(right, 0, 'f', 2));
      qDebug() << "Values changed - Left:" << left << "Right:" << right;
    });

  window.setCentralWidget(central);
  window.resize(600, 400);
  window.show();

  return app.exec();
}

