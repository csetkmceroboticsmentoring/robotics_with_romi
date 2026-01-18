
#include "romi_robot_server.h"

#include <QDebug>
#include <QDataStream>

RomiRobotServer::RomiRobotServer(const QString& ip_addr, const uint16_t port)
    : QObject(nullptr) {
  timer = new QTimer(this);
  tcp_server = new QTcpServer(this);
  motion_controller = new MotionControlThread(this, mutex, queue);
  tcp_server->setMaxPendingConnections(1);
  const bool status = tcp_server->listen(QHostAddress(ip_addr), port);
  if (!status) {
    qDebug() << "Error: " << tcp_server->errorString();
    exit(0);
  }
  qDebug() << "Port: " << tcp_server->serverPort();
  connect(timer, SIGNAL(timeout()), this, SLOT(timeout()));
  connect(tcp_server, SIGNAL(newConnection()), this, SLOT(newConnection()));

  rx_data.reserve(1000);
  position_cache.reserve(1000);

  timer->start(25);
  motion_controller->start();
}

void RomiRobotServer::newConnection() {
  auto tcp_socket = tcp_server->nextPendingConnection();
  connect(tcp_socket, SIGNAL(readyRead()), this, SLOT(readReady()));
  //connect(tcp_socket, SIGNAL(QAbstractSocket::disconnected()), tcp_socket, SLOT(QObject::deleteLater()));
  const auto state = motion_controller->state();
  const QString str = QString("s: %1, %2, %3").arg(state.x()).arg(state.y()).arg(state.z());
  tcp_socket->write(str.toUtf8());
}

void RomiRobotServer::readReady() {
  auto tcp_sockets = findChildren<QTcpSocket*>();
  double x, y;
  for (auto& tcp_socket : tcp_sockets) {
    if (tcp_socket->bytesAvailable() > 0) {
      const int size = tcp_socket->read(rx_data.data(), rx_data.capacity());
      rx_data[size] = '\0';
      sscanf(rx_data.data(), "w: %lf, %lf", &x, &y);
      motion_controller->addWayPoints(x, y);
    }
  }
}

void RomiRobotServer::timeout() {
  position_cache.clear();
  {
    QMutexLocker lock(&mutex);
    while (!queue.empty()) {
      position_cache.push_back(queue.front());
      queue.pop_front();
    }
  }
  auto tcp_sockets = findChildren<QTcpSocket*>();
  for (auto& tcp_socket : tcp_sockets) {
    for (const auto& pos : position_cache) {
      const QString str = QString("s: %1, %2, %3").arg(pos.x()).arg(pos.y()).arg(pos.z());
      tcp_socket->write(str.toUtf8());
    }
  }
}
