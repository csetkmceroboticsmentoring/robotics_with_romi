#include "motion_control_thread.h"

#include <QDebug>

#include <Eigen/Core>
#include <cmath>
#include <random>

#include "pid_control.h"
#include "wheel_with_encoder.h"

using Eigen::Vector2d;
using Eigen::Vector3d;

// Robot physical parameters
constexpr double kPI = 3.14159265358979323846;
constexpr double kWheelDiameter = 0.07;
constexpr double kDistanceBetweenWheels = 0.14;
constexpr uint32_t kEncoderTicksPerRotation = 1440;

double radToDegree(const double theta) {
  return (180.0 * theta) / kPI;
}

double encoderTicksToDistance(const double enc_ticks) {
  return (kWheelDiameter * kPI / kEncoderTicksPerRotation) * enc_ticks;
}

double distanceToEncoderTicks(const double dist) {
  return std::round((kEncoderTicksPerRotation * dist) / (kWheelDiameter * kPI));
}

double angleToEncoderTicks(const double theta_rad) {
  return distanceToEncoderTicks(kDistanceBetweenWheels * theta_rad);
}

double encoderTicksToAngle(const double enc_ticks) {
  return encoderTicksToDistance(enc_ticks) / kDistanceBetweenWheels;
}

// Calculate signed angle difference between current heading and destination direction
double headingDiff(const double& heading, const Vector2d& dest_dir) {
  if (dest_dir.x() == 0.0 && dest_dir.y() == 0.0) {
    return 0.0;
  }
  const Vector2d head_dir(cos(heading), sin(heading));
  const double cross_prod = head_dir.x()*dest_dir.y() - dest_dir.x()*head_dir.y();
  const double angle = std::acos(head_dir.dot(dest_dir)/dest_dir.norm());
  return (cross_prod < 0.0)?(-angle):(angle);
}

template <typename T>
T clamp(const T val, const T min_range, const T max_range) {
  return std::min(std::max(min_range, val), max_range);
}

MotionControlThread::MotionControlThread(QObject* parent, QMutex& mutex, QQueue<WayPoint>& queue)
  : QThread(parent),
    heading(0.0),
    pos(0.0, 0.0),
    mutex(mutex),
    queue(queue) {}

MotionControlThread::~MotionControlThread() {
  stop();
}

void MotionControlThread::run() {
  const double dt = 0.025;  // Control loop period (25ms)

  PidControl heading_pid(0.125, 0.0, 0.01);
  PidControl distance_pid(1.5, 0.0, 0.01);

  WheelWithEncoder left_wheel(0.1613, 12.9032, 0.0);
  WheelWithEncoder right_wheel(0.1613, 12.9032, 0.0);

  heading_pid.setTarget(0.0f);
  distance_pid.setTarget(0.0f);

  WayPoint dest(0.0, 0.0, 0.0);

  done = false;

  double heading_diff;

  // Main control loop at ~40Hz
  auto start_time = std::chrono::high_resolution_clock::now();
  while (!done) {
    auto curr_time = std::chrono::high_resolution_clock::now();
    const auto diff_time = curr_time - start_time;
    if (diff_time < std::chrono::milliseconds(25)) {
      continue;
    }
    start_time = curr_time;
    
    // Calculate distance and heading error to destination
    double d = (dest.pos - pos).norm();
    heading_diff = headingDiff(heading, (dest.pos-pos));

    // Reached destination, get next waypoint
    if (d < 0.02) {
      QMutexLocker lock(&cond_mutex);
      if (way_points.empty()) {
        cond_wait.wait(&cond_mutex);
      } else {
        dest = way_points.front();
        way_points.pop_front();
        qDebug() << "dest: " << dest.pos.x() << ", " << dest.pos.y() << ", " << radToDegree(heading_diff);
      }
      continue;
    }

    // Compute wheel velocities using PID control
    {
      // PID controllers have target set to 0.0, so we pass negative error to get positive correction
      const int dist_corr = distance_pid.process(-distanceToEncoderTicks(d), dt, -50, +50);
      const int correction = heading_pid.process(-angleToEncoderTicks(heading_diff), dt, -50, +50);
      right_wheel.setTarget(clamp<int>(dist_corr*std::abs(cos(heading_diff)) + correction, -50, +50));
      left_wheel.setTarget(clamp<int>(dist_corr * std::abs(cos(heading_diff)) - correction, -50, +50));
    }

    // Update odometry from wheel encoder velocities
    const int lv = left_wheel.update();
    const int rv = right_wheel.update();
    const double cv = static_cast<double>(lv + rv) / 2.0;
    heading += encoderTicksToAngle(rv - lv);
    const double dist = encoderTicksToDistance(cv);
    pos += Vector2d(dist * cos(heading), dist * sin(heading));

    // Publish current state
    {
      QMutexLocker lock(&mutex);
      queue.push_back(WayPoint(pos.x(), pos.y(), heading));
    }
  }
  qDebug() << "Done ...";
}

void MotionControlThread::stop() {
  done = true;
  cond_wait.notify_all();
  wait();
}

void MotionControlThread::addWayPoints(const double x, const double y) {
  {
    QMutexLocker lock(&cond_mutex);
    way_points.push_back(WayPoint(x, y, 0));
  }
  cond_wait.notify_all();  // Wake up control loop if waiting for waypoints
}

Eigen::Vector3d MotionControlThread::state() {
  Eigen::Vector3d head;
  {
    QMutexLocker lock(&mutex);
    head = Vector3d(pos.x(), pos.y(), heading);
  }
  return head;
}
