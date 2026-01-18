#include <QCoreApplication>

#include "romi_robot_server.h"

int main(int argc, char *argv[]) {
  QCoreApplication app(argc, argv);
  RomiRobotServer robot_server(QString("localhost"), 60000);
  return app.exec();
}
