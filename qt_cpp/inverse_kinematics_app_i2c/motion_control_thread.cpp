#include "motion_control_thread.h"
#include "i2c_slave.h"
#include "cmd_response.h"

#include <QDebug>
#include <Eigen/Core>
#include <cmath>

using Eigen::Vector2d;
using Eigen::Vector3d;

class RomiRobot {
public:
  struct State {
    double x;
    double y;
    double yaw;
    bool is_wps_q_full;
    double timestamp;
  };

  RomiRobot(const QString& i2c_device, uint8_t slave_address) 
    : i2c_slave_(i2c_device, slave_address) {
    if (!i2c_slave_.open()) {
      qDebug() << "Failed to open I2C device: " << i2c_device;
      exit(1);
    }
    qDebug() << "I2C communication initialized";
  }

  bool setWayPoint(const double x, const double y) {
    // Convert meters to millimeters (multiply by 1000) and cast to int32_t
    const int32_t xi = static_cast<int32_t>(std::ceil(x * 1000.0));
    const int32_t yi = static_cast<int32_t>(std::ceil(y * 1000.0));
    if (!i2c_slave_.sendWaypointCommand(xi, yi)) {
      qDebug() << "Failed to send waypoint command: x=" << x << "y=" << y;
      return false;
    }
    return true;
  }

  State getCurrentState() {
    State state;
    struct Status status;
    if (i2c_slave_.getStatus(status)) {
      state.x = static_cast<double>(status.x) / 1000.0;
      state.y = static_cast<double>(status.y) / 1000.0;
      state.yaw = static_cast<double>(status.yaw) / 1000.0;
      state.is_wps_q_full = (status.is_queue_full != 0);
      state.timestamp = static_cast<double>(status.time_stamp) / 1000.0; // Convert from milliseconds to seconds
    } else {
      qDebug() << "Failed to get status from I2C";
    }
    return state;
  }

private:
  I2cSlave i2c_slave_;
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
  // Default I2C device path and slave address (Arduino uses address 20)
  RomiRobot romi(QString("/dev/i2c-1"), 20);

  RomiRobot::State state = romi.getCurrentState();
  {
    QMutexLocker lock(&mutex);
    queue.push_back(Vector3d(state.x, state.y, state.yaw));
  }

  Vector2d dest(state.x, state.y);
  
  done = false;

  while (!done) {
    state = romi.getCurrentState();
    
    // Get waypoint from queue (minimal lock time)
    bool has_waypoint = false;
    Vector2d waypoint_to_send;
    {
      QMutexLocker lock(&cond_mutex);
      if (!way_points.empty() && !state.is_wps_q_full) {
        waypoint_to_send = way_points.front();
        has_waypoint = true;
      }
    }
    
    // Send waypoint without holding the lock (I2C operation can take time)
    if (has_waypoint) {
      if (romi.setWayPoint(waypoint_to_send.x(), waypoint_to_send.y())) {
        // Remove from queue after successful send
        QMutexLocker lock(&cond_mutex);
        if (!way_points.empty() && way_points.front().x() == waypoint_to_send.x() && 
            way_points.front().y() == waypoint_to_send.y()) {
          way_points.pop_front();
        }
        qDebug() << "✓ Waypoint sent successfully:" << waypoint_to_send.x() << ", " << waypoint_to_send.y();
      } else {
        qDebug() << "✗ Failed to send waypoint, will retry:" << waypoint_to_send.x() << ", " << waypoint_to_send.y();
      }
    }
    
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
