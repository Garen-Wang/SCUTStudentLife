#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QPen>

#include <QBrush>
#include <QPainter>
#include <QPixmap>
#include <QPoint>

#include <QDebug>
#include <QHBoxLayout>
#include <QIODevice>
#include <QPaintEvent>
#include <QPoint>
#include <QPushButton>

#include "loader.h"
#include "map.h"

void MainWindow::createRealAgents() {
  AgentLoader agentLoader(socialForce);
  agentLoader.generateFromJsonFile("21软件工程1班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("21软件工程2班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("21软件工程3班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("21软件工程4班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("21软件工程中澳班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("21软件工程卓越班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("20软件工程1班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("20软件工程2班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("20软件工程3班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("20软件工程4班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("20软件工程中澳班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("20软件工程卓越班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("19软件工程1班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("19软件工程2班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("19软件工程3班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("19软件工程4班课表.json", 100, "C10", "C5");
  agentLoader.generateFromJsonFile("19软件工程中澳班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("19软件工程卓越班课表.json", 100, "C10",
                                   "C5");

  agentLoader.generateFromJsonFile("20计算机科学与技术1班课表.json", 100, "C12",
                                   "C5");
  agentLoader.generateFromJsonFile("20计算机科学与技术2班课表.json", 100, "C12",
                                   "C5");
  agentLoader.generateFromJsonFile("20计科全英创新班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("20计科全英联合班课表.json", 100, "C6",
                                   "C5");
  agentLoader.generateFromJsonFile("20网络工程课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("20信息安全课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("19计算机科学与技术1班课表.json", 100, "C12",
                                   "C5");
  agentLoader.generateFromJsonFile("19计算机科学与技术2班课表.json", 100, "C12",
                                   "C5");
  agentLoader.generateFromJsonFile("19计科全英创新班课表.json", 100, "C10",
                                   "C5");
  agentLoader.generateFromJsonFile("19计科全英联合班课表.json", 100, "C6",
                                   "C5");
  agentLoader.generateFromJsonFile("19网络工程课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("19信息安全课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("21计科全英创新班课表.json", 100, "C12",
                                   "C5");
  agentLoader.generateFromJsonFile("21计科全英联合班课表.json", 100, "C12",
                                   "C5");
  agentLoader.generateFromJsonFile("21计算机类1班课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("21计算机类2班课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("21计算机类3班课表.json", 100, "C12", "C5");
  agentLoader.generateFromJsonFile("21计算机类4班课表.json", 100, "C12", "C5");

  //    agentLoader.generateAll();
  qDebug() << socialForce->getAgents().size();
  //    socialForce->initWaitingList(numWeek, dayOfWeek);
}

void MainWindow::createWallsFromReadEdges(
    const std::vector<ReadEdge> &vec_walls) {
  for (auto it : vec_walls) {
    auto wall = new Wall(it.first.first, it.first.second, it.second.first,
                         it.second.second);
    socialForce->addWall(wall);
  }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), offsetX(0), offsetY(0),
      scale(1.0), currentTime(7, 50, 0), numWeek(1), dayOfWeek(1),
      preClock(Clock::now()), rate(4), numAgents(0), animate(false) {
  ui->setupUi(this);
  timerId = startTimer(33);
  socialForce = new SocialForceModel();

  initMap();                       // map.init()
  createRealAgents();              // agent builder
  createWallsFromReadEdges(walls); // wall builder
}

MainWindow::~MainWindow() {
  delete ui;
  socialForce->removeAllAgents();
  socialForce->removeAllWalls();
  delete socialForce;
}

void MainWindow::paintEvent(QPaintEvent *event) {
  auto sz = size();
  int temp = static_cast<int>(sz.height() * 0.8);
  int left = 120, right = sz.width() - 50 - 80;
  ui->playButton->move(left, temp + 20);
  left -= 80;

  ui->banBridgeButton1->move(left, temp + 60);
  left += 50 + 150;

  ui->banBridgeButton2->move(left, temp + 60);
  //  left += 50 + 80;

  ui->setRateButton->move(right, temp + 20);
  ui->rateEdit->move(right - 50 - 80, temp + 20);

  ui->setWeekButton->move(right, temp + 80);
  ui->weekEdit->move(right - 50 - 80, temp + 80);

  ui->setDayOfWeekButton->move(right, temp + 80 + 60);
  ui->dayOfWeekEdit->move(right - 50 - 80, temp + 80 + 60);

  ui->timeLabel->move(right - 200, 60);
  ui->numberOfAgentsLabel->move(right - 200, 60 + 60);
  ui->rateLabel->move(right - 200, 60 + 120);

  ui->timeButton->move(right - 300, temp + 20);
  ui->timeEdit->move(right - 450, temp + 20);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  QTransform transform;
  transform.scale(scale, scale);

  int side = static_cast<int>(qMin(sz.width(), sz.height()) * 0.8 / scale);

  QSize ViewWH(side, side);

  painter.setViewport(side / 2, side / 2, side, side);
  painter.setWindow(0, 0, side, -side);

  painter.setBrush(QColor(187, 229, 253));

  painter.setPen(QPen(QColor(11, 67, 127), 2));

  // draw walls
  double x_max = getMaxX(), x_min = getMinX();
  double y_max = getMaxY(), y_min = getMinY();
  for (auto wall : socialForce->getWalls()) {
    QPointF a(((wall->getStartPoint().x - x_min) / (x_max - x_min) - 0.5) *
                      ViewWH.width() +
                  offsetX,
              ((wall->getStartPoint().y - y_min) / (y_max - y_min) - 0.5) *
                      ViewWH.height() +
                  offsetY);
    QPointF b(((wall->getEndPoint().x - x_min) / (x_max - x_min) - 0.5) *
                      ViewWH.width() +
                  offsetX,
              ((wall->getEndPoint().y - y_min) / (y_max - y_min) - 0.5) *
                      ViewWH.height() +
                  offsetY);
    painter.drawLine(a, b);
  }

  // draw agents
  for (auto agent : socialForce->getAgents()) {
    if (!agent->isFinished()) {
      QPointF a(
          ((agent->getX() - x_min) / (x_max - x_min) - 0.5) * ViewWH.width() +
              offsetX,
          ((agent->getY() - y_min) / (y_max - y_min) - 0.5) * ViewWH.height() +
              offsetY);
      if (agent->getTransport() == OnFoot) {
        painter.setPen(QPen(Qt::green, 10, Qt::SolidLine, Qt::RoundCap));
      } else if (agent->getTransport() == Bicycle) {
        painter.setPen(QPen(Qt::blue, 10, Qt::SolidLine, Qt::RoundCap));
      } else if (agent->getTransport() == ElectricBicycle) {
        painter.setPen(QPen(Qt::red, 10, Qt::SolidLine, Qt::RoundCap));
      } else {
        assert(false);
      }
      painter.drawPoint(a);
    }
  }

  numAgents = socialForce->getAgents().size();
  ui->rateLabel->setText("rate: " + QString::number(rate));
  ui->timeLabel->setText(
      QString("time: %1:%2:%3 | %4")
          .arg(QString::number(currentTime.h),
               QString::number(currentTime.m).rightJustified(2, '0'),
               QString::number(currentTime.s).rightJustified(2, '0'),
               QString::number(dayOfWeek)));
  ui->numberOfAgentsLabel->setText("agents: " + QString::number(numAgents));

  // update();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
  case Qt::Key_J:
    // down
    qDebug() << "j pressed";
    offsetY -= 50;
    break;
  case Qt::Key_K:
    // up
    qDebug() << "k pressed";
    offsetY += 50;
    break;
  case Qt::Key_H:
    // left
    qDebug() << "h pressed";
    offsetX -= 50;
    break;
  case Qt::Key_L:
    // right
    qDebug() << "l pressed";
    offsetX += 50;
    break;
  case Qt::Key_Plus:
    qDebug() << "+ pressed";
    scale -= 0.05;
    break;
  case Qt::Key_Minus:
    qDebug() << "- pressed";
    scale += 0.05;
    break;
  }
};

void MainWindow::on_setRateButton_clicked() {
  int newRate = ui->rateEdit->text().toInt();
  rate = newRate;
}

void MainWindow::timerEvent(QTimerEvent *event) {
  auto nowClock = Clock::now();
  auto duration = preClock.time_since_epoch();
  if (duration.zero() != duration) {
    auto elapsed = nowClock - preClock;
    int t =
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

    if (animate) {
      currentTime += t * rate;
      socialForce->move(rate * t * 0.001, currentTime);
      socialForce->checkWaitingList(currentTime);
    }
  }
  preClock = nowClock;
  update();
}

void MainWindow::on_playButton_clicked() {
  animate = !animate;
  ui->playButton->setText(animate ? "pause" : "play");
}

void MainWindow::on_setWeekButton_clicked() {
  auto text = ui->weekEdit->text();
  int numWeek = text.toInt();
  assert(numWeek >= 1);
  this->numWeek = numWeek;
}

void MainWindow::on_setDayOfWeekButton_clicked() {
  auto text = ui->dayOfWeekEdit->text();
  int dayOfWeek = text.toInt();
  assert(dayOfWeek >= 0 && dayOfWeek < 7);
  this->dayOfWeek = dayOfWeek;
}

void MainWindow::on_timeButton_clicked() {
  auto time = ui->timeEdit->time();
  currentTime.set(time.hour(), time.minute(), time.second());
  socialForce->initWaitingList(numWeek, dayOfWeek, currentTime);
}

void MainWindow::on_banBridgeButton1_clicked() {
  setBan1(!getBan1());
  ui->banBridgeButton1->setText(getBan1() ? "undo only1" : "only bridge1");
}

void MainWindow::on_banBridgeButton2_clicked() {
  setBan2(!getBan2());
  ui->banBridgeButton2->setText(getBan2() ? "undo only2" : "only bridge2");
}
