#pragma once

#include <QDialog>
#include <QTimer>
#include <QMutex>
#include <QQueue>

#include "comms_thread.h"

#include "ui_main_dlg.h"

class MainDlg : public QDialog, private Ui::MainDlgUi {
  Q_OBJECT
public:
  MainDlg();

private slots:
  void timeout();
  void connectToRobot();
  void addWayPoints(const double x, const double y);

private:
  QTimer timer;
  CommsThread* comms_thread = nullptr;

  QMutex mutex;
  QQueue<Eigen::Vector3d> queue;

  std::vector<Eigen::Vector2f> points;
};
