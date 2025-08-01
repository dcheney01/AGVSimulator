#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>
#include <iostream>

#include "robot.hpp"
#include "environment.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void on_timer();
    void on_start();
    void on_reset();
    void on_pause();
    void on_setRobot();
    void on_astarSolve();
    void on_setNewGoalLocation();
    void on_updateEnvironment();
    void on_generateObstacles();

protected:
    void connect_buttons();
    void create_scene();
    void create_robot_graphics();
    void create_timer();
    void set_dependent_text();

private:
    Ui::MainWindow *ui{nullptr};
    QGraphicsPolygonItem* robotGraphicsItem{nullptr};
    QGraphicsPolygonItem* goalLocationGraphicsItem{nullptr};
    QTimer* timer{nullptr};
    QGraphicsScene* scene{nullptr};

    const double defaultBotX{5.0};
    const double defaultBotY{5.0};
    const double defaultBotTheta{0.0};

    Environment env{900, 790};
    Robot robot{defaultBotX, defaultBotY, defaultBotTheta};

    std::vector<std::array<double,2>> path{};
    std::vector<std::array<double,2>> obstacles{};

    const double SIM_HZ{100.0};
    const double CONTROL_HZ{10.0};
    const int obstacleSize{10};
    const double obsTolerance{1};
    const int pathSparsity{5};
};
#endif // MAINWINDOW_H
