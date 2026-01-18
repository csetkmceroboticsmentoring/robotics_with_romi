#include <QApplication>

#include "main_dlg.h"

int main(int argc, char** argv) {
  QApplication app(argc, argv);  
  MainDlg dlg;
  dlg.show();
  return app.exec();
}
