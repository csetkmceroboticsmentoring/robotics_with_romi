#pragma once

#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QWaitCondition>

#include <Eigen/Core>

class MotionControlThread : public QThread {
  Q_OBJECT
public:
  MotionControlThread(QObject* parent, QMutex& mutex, QQueue<Eigen::Vector3d>& queue);
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
  QQueue<Eigen::Vector3d>& queue;

  QMutex cond_mutex;
  QWaitCondition cond_wait;
  QQueue<Eigen::Vector2d> way_points;
};
