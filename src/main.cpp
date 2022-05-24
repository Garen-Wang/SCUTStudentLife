#include "mainwindow.h"

#include <QApplication>
#include <QDebug>

int main(int argc, char *argv[]) {
  srand(19260817);
  QApplication a(argc, argv);
  MainWindow w;
  w.resize(1000, 1000);
  w.show();
  return a.exec();
}
