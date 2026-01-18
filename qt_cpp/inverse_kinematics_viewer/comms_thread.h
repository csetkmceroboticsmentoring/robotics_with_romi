#pragma once

#include <QQueue>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include <QTcpSocket>

#include <Eigen/Core>

class CommsThread : public QThread {
  Q_OBJECT
public:
  CommsThread(QObject* parent, const QString& host_name, QMutex& mutex, QQueue<Eigen::Vector3d>& queue);
  ~CommsThread();

  void stop();

  void sendWayPoint(const double x, const double y);

private:
  void run() override;

  bool done = false;

  const QString host_name;

  QQueue<Eigen::Vector2d> way_points;

  QMutex& mutex;
  QQueue<Eigen::Vector3d>& queue;
  
  QTcpSocket* tcp_socket;
};
