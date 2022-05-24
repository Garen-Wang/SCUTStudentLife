#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPaintEvent>
#include <chrono>

#include "config.h"
#include "social-force.h"

using Clock = std::chrono::high_resolution_clock;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override;
  void paintEvent(QPaintEvent *) override;

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void timerEvent(QTimerEvent *event) override;

private slots:
  void on_playButton_clicked();

  void on_setRateButton_clicked();

  void on_setWeekButton_clicked();

  void on_setDayOfWeekButton_clicked();

  void on_timeButton_clicked();

  void on_banBridgeButton1_clicked();

  void on_banBridgeButton2_clicked();

private:
  Ui::MainWindow *ui;

  int offsetX;
  int offsetY;
  double scale;
  int timerId;
  SocialForceModel *socialForce;
  bool animate;

  Time currentTime;
  int numWeek;
  int rate;
  int hour, minute, second;
  int dayOfWeek;
  int numAgents;
  std::chrono::time_point<Clock> preClock;

  void createRealAgents();
  void createWallsFromReadEdges(const std::vector<ReadEdge> &vec_walls);
};
#endif // MAINWINDOW_H
