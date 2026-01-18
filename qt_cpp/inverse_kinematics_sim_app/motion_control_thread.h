#pragma once

#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QWaitCondition>

#include <Eigen/Core>

struct WayPoint {
  WayPoint(const double x, const double y, const double yaw) : pos(x, y), yaw(yaw) {}

  Eigen::Vector2d pos;
  double yaw;
};

class MotionControlThread : public QThread {
  Q_OBJECT
public:
  MotionControlThread(QObject* parent, QMutex& mutex, QQueue<WayPoint>& queue);
  ~MotionControlThread();

  void stop();

  void addWayPoints(const double x, const double y);

  Eigen::Vector3d state();
  
private:
  void run() override;

  bool done = false;

  double heading;
  Eigen::Vector2d pos;

  QMutex& mutex;
  QQueue<WayPoint>& queue;

  QMutex cond_mutex;
  QWaitCondition cond_wait;
  QQueue<WayPoint> way_points;
};
