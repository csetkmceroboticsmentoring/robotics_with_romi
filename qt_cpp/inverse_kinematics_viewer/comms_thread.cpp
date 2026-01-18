#include "comms_thread.h"
  
CommsThread::CommsThread(QObject* parent, const QString& host_name, QMutex& mutex, QQueue<Eigen::Vector3d>& queue)
  : QThread(parent),
    host_name(host_name),
    mutex(mutex),
    queue(queue) {}

CommsThread::~CommsThread() {
  stop();
}

void CommsThread::stop() {
  done = true;
  wait();
}

void CommsThread::sendWayPoint(const double x, const double y) {
  QMutexLocker lock(&mutex);
  way_points.push_back(Eigen::Vector2d(x, y));
}

void CommsThread::run() {
  tcp_socket = new QTcpSocket;
  tcp_socket->connectToHost(host_name, 60000, QIODevice::ReadWrite, QAbstractSocket::IPv4Protocol);

  QByteArray rx_data(1000, '\0');
  done = false;
  
  while (!done) {
    if (tcp_socket->state() == QAbstractSocket::UnconnectedState) {
      done = true;
      continue;
    }
    {
      QMutexLocker lock(&mutex);
      while (!way_points.empty()) {
        const auto wp = way_points.front();
        way_points.pop_front();
        const QString w_str = QString("w: %1, %2").arg(wp.x()).arg(wp.y());
        tcp_socket->write(w_str.toUtf8());
      }
    }

    const bool status = tcp_socket->waitForReadyRead(100);
    if (!status) {
      continue;
    }

    double x, y, yaw;
    const int size = tcp_socket->read(rx_data.data(), rx_data.capacity());
    rx_data[size] = '\0';
    sscanf(rx_data.data(), "s: %lf, %lf, %lf", &x, &y, &yaw);
    {
      QMutexLocker lock(&mutex);
      queue.push_back(Eigen::Vector3d(x, y, yaw));
    }
  }
}
