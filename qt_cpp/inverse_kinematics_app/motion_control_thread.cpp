#include "motion_control_thread.h"

#include <QDebug>
#include <QByteArray>
#include <QSerialPort>

#include <Eigen/Core>
#include <cmath>
#include <random>

using Eigen::Vector2d;
using Eigen::Vector3d;

class RomiRobot {
public:
  struct Gain {
    long int kp;
    long int ki;
    long int kd;
  };
  struct State {
    double x;
    double y;
    double yaw;
    bool is_idle;
    bool is_wps_q_full;
    double timestamp;
  };

  RomiRobot(const QString& port_name) {
    rx_buffer.reserve(1000);
    tx_buffer.reserve(1000);
    serial_port.setPortName(port_name);
    serial_port.setBaudRate(QSerialPort::Baud115200);
    serial_port.setDataBits(QSerialPort::Data8);
    serial_port.setFlowControl(QSerialPort::NoFlowControl);
    serial_port.setParity(QSerialPort::NoParity);
    serial_port.open(QIODevice::ReadWrite);
    if (!serial_port.isOpen()) {
      qDebug() << "Failed to open serial port: " << serial_port.portName() << "\n";
      exit(0);
    }
  }

  void setGains(const Gain& gains_v, const Gain& gains_d, const Gain& gains_y) {
    const int num_chars = 
      sprintf(tx_buffer.data(),
              "k %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
              gains_v.kp, gains_v.ki, gains_v.kd,
              gains_d.kp, gains_d.ki, gains_d.kd,
              gains_y.kp, gains_y.ki, gains_y.kd);
    if (num_chars < 0) {
      qDebug() << "SetGains Failed";
      exit(0);
    }
    serial_port.write(tx_buffer.data());
    serial_port.waitForBytesWritten();
    serial_port.waitForReadyRead(2000);
    const int bytes_read = serial_port.read(rx_buffer.data(), rx_buffer.capacity());
    rx_buffer[bytes_read] = '\0';
    qDebug() << bytes_read << " - " << rx_buffer.data();
  }

  void setWayPoint(const double x, const double y) {
    const int xi = static_cast<int>(std::ceil(x*1000.0));
    const int yi = static_cast<int>(std::ceil(y*1000.0));
    const int num_chars = sprintf(tx_buffer.data(), "w %d, %d\r\n", xi, yi);
    if (num_chars < 0) {
      qDebug() << "SetWayPoint Failed";
      exit(0);
    }
    serial_port.write(tx_buffer.data());
    serial_port.waitForBytesWritten();
    serial_port.waitForReadyRead(2000);
    const int bytes_read = serial_port.read(rx_buffer.data(), rx_buffer.capacity());
    rx_buffer[bytes_read] = '\0';
    qDebug() << bytes_read << " - " << rx_buffer.data();
  }

  State getCurrentState() {
    State state;
    const int num_chars = sprintf(tx_buffer.data(), "s\r\n");
    if (num_chars < 0) {
      qDebug() << "SetWayPoint Failed";
      exit(0);
    }
    serial_port.write(tx_buffer.data());
    serial_port.waitForBytesWritten();
    serial_port.waitForReadyRead(2000);
    const int bytes_read = serial_port.read(rx_buffer.data(), rx_buffer.capacity());
    rx_buffer[bytes_read] = '\0';
    unsigned long int timestamp;
    int is_idle, is_wps_q_full;
    int x, y, yaw;
    sscanf(rx_buffer.data(), "s: %d, %d, %d, %lu, %d, %d\r\n",
      &x, &y, &yaw, &timestamp, &is_wps_q_full, &is_idle);
    state.x = static_cast<double>(x) / 1000.0f;
    state.y = static_cast<double>(y) / 1000.0f;
    state.yaw = static_cast<double>(yaw) / 1000.0f;
    qDebug() << bytes_read << " - " << rx_buffer.data();
    state.timestamp = static_cast<double>(timestamp) / 1000.0;
    state.is_wps_q_full = (bool)is_wps_q_full;
    state.is_idle = (bool)is_idle;
    return state;
  }

private:
  QByteArray rx_buffer;
  QByteArray tx_buffer;
  QSerialPort serial_port;
};

MotionControlThread::MotionControlThread(QObject* parent, QMutex& mutex, QQueue<Eigen::Vector3d>& queue)
   : QThread(parent),
	heading(0.0),
  pos(0.0, 0.0),
  mutex(mutex),
	queue(queue) {}

MotionControlThread::~MotionControlThread() {
	stop();
}

void MotionControlThread::run() {
  // For Windows OS Replace the port name as COMX (Ex: COM1, COM2 etc)
  RomiRobot romi("/dev/ttyACM0");

  const RomiRobot::Gain gains_v = { 161, 12903, 0 };
  const RomiRobot::Gain gains_d = { 1500, 0, 10 };
  const RomiRobot::Gain gains_y = { 125, 0, 10 };

  romi.setGains(gains_v, gains_d, gains_y);

  RomiRobot::State state = romi.getCurrentState();
  {
    QMutexLocker lock(&mutex);
    queue.push_back(Vector3d(state.x, state.y, state.yaw));
  }

  Vector2d dest(state.x, state.y);

  done = false;

  while (!done) {
    {
      QMutexLocker lock(&cond_mutex);
      if (way_points.empty()) {
        /*if (state.is_idle) {
          cond_wait.wait(&cond_mutex);
        }*/
      } else {
        if (!state.is_wps_q_full) {
          dest = way_points.front();
          way_points.pop_front();
          romi.setWayPoint(dest.x(), dest.y());
          qDebug() << "---- Here: " << dest.x() << ", " << dest.y();
        }
      }
    }

    state = romi.getCurrentState();
    {
      QMutexLocker lock(&mutex);
      queue.push_back(Vector3d(state.x, state.y, state.yaw));
    }
    msleep(100);
  }
}

void MotionControlThread::stop() {
	done = true;
  cond_wait.notify_all();
	wait();
}

void MotionControlThread::addWayPoints(const double x, const double y) {
  {
    QMutexLocker lock(&cond_mutex);
    way_points.push_back(Eigen::Vector2d(x, y));
  }
  cond_wait.notify_all();
}

Eigen::Vector3d MotionControlThread::state() {
  Eigen::Vector3d head;
  {
    QMutexLocker lock(&mutex);
    head = Vector3d(pos.x(), pos.y(), heading);
  }
  return head;
}
