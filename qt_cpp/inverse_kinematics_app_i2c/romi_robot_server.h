#pragma once

#include <QMutex>
#include <QQueue>
#include <QTimer>
#include <QObject>
#include <QByteArray>
#include <QTcpSocket>
#include <QTcpServer>

#include "motion_control_thread.h"

class RomiRobotServer : public QObject {
  Q_OBJECT
public:
  RomiRobotServer(const QString& ip_addr, const uint16_t port);

private slots:
  void timeout();
  void readReady();
  void newConnection();

private:
  QTimer* timer;
  QTcpServer* tcp_server;

  QMutex mutex;
  QQueue<Eigen::Vector3d> queue;
  MotionControlThread* motion_controller;

  QByteArray rx_data;
  std::vector<Eigen::Vector3d> position_cache;
};
