#include "main_dlg.h"

#include <QDebug>

MainDlg::MainDlg()
  : QDialog(nullptr, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint | Qt::WindowMinimizeButtonHint) {
  setupUi(this);
  points.reserve(1000);
  connect(connect_btn, SIGNAL(clicked()), this, SLOT(connectToRobot()));
  connect(&timer, SIGNAL(timeout()), this, SLOT(timeout()));
  connect(path_view, SIGNAL(newWayPoint(const double, const double)), 
    this, SLOT(addWayPoints(const double, const double)));
  timer.setInterval(25);
  timer.start();
  path_view->setFocus();
}

void MainDlg::timeout() {
  float yaw = 0.0f;
  {
    QMutexLocker lock(&mutex);
    points.resize(0);
    while (!queue.empty()) {
      const auto item = queue.front();
      yaw = static_cast<float>(item.z());
      points.emplace_back(Eigen::Vector2f(item.x(), item.y()));
      queue.pop_back();
    }
  }
  if (!points.empty()) {
    path_view->add(yaw, points);
  }
  
  if (comms_thread) {
    path_view->setWaypointEnabled(comms_thread->isRunning());
    host_ip->setDisabled(comms_thread->isRunning());
    connect_btn->setDisabled(comms_thread->isRunning());
  } else {
    path_view->setWaypointEnabled(false);
  }
}

void MainDlg::addWayPoints(const double x, const double y) {
  if (comms_thread) {
    comms_thread->sendWayPoint(x, y);
  }
}

void MainDlg::connectToRobot() {
  comms_thread = new CommsThread(this, host_ip->text(), mutex, queue);
  comms_thread->start();
  host_ip->setDisabled(true);
  connect_btn->setDisabled(true);
  path_view->clear();
  path_view->setFocus();
}
