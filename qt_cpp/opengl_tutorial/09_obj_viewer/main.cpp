/**
 * @file main.cpp
 * @brief Lesson 9 — OBJ viewer (shuttle.obj loaded in WidgetGL::initializeGL).
 *
 * Drag with the left mouse button to rotate the model (trackball).
 */

#include <QApplication>
#include <QLabel>
#include <QMainWindow>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWidget>

#include "widget_gl.h"

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  format.setDepthBufferSize(24);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);

  QMainWindow window;
  window.setWindowTitle(QStringLiteral("Lesson 9"));

  auto* central = new QWidget(&window);
  auto* layout = new QVBoxLayout(central);
  layout->setContentsMargins(8, 8, 8, 8);

  auto* hint = new QLabel(QStringLiteral("Hold left mouse button inside and drag"), central);
  hint->setAlignment(Qt::AlignCenter);
  layout->addWidget(hint);

  layout->addWidget(new WidgetGL(central), 1);

  window.setCentralWidget(central);
  window.resize(960, 720);
  window.show();
  return app.exec();
}
