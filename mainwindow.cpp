#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTimer>
#include <QElapsedTimer>
#include <QMessageBox>

#include "drawing_utils.hpp"
#include "astar.hpp"
#include "math_utils.hpp"
#include "path_follower.hpp"
#include "obstacle_utils.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    create_scene();
    create_timer();
    connect_buttons();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::create_scene()
{
    scene = new QGraphicsScene(ui->simulatorView);
    scene->setSceneRect(QRectF(0, 0, env.get_xSize(), env.get_ySize()));

    robotGraphicsItem = draw::draw_triangle(robot.get_pose());
    scene->addItem(robotGraphicsItem);

    goalLocationGraphicsItem = draw::draw_star(env.get_goalLocation());
    scene->addItem(goalLocationGraphicsItem);

    draw::draw_obstacles_in_scene(scene, obstacles, obstacleSize);

    ui->simulatorView->setScene(scene);
    set_dependent_text();
}

void MainWindow::set_dependent_text()
{   
    ui->goalXTextEdit->setPlainText(QString::number(env.get_goalLocation()[0]));
    ui->goalYTextEdit->setPlainText(QString::number(env.get_goalLocation()[1]));

    ui->xPosTextEdit->setPlainText(QString::number(robot.get_x()));
    ui->yPosTextEdit->setPlainText(QString::number(robot.get_y()));
    ui->thetaTextEdit->setPlainText(QString::number(my_math::rad2deg(robot.get_theta())));
}

void MainWindow::create_timer()
{
    timer = new QTimer(this);
    timer->setInterval(CONTROL_HZ);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(on_timer()));
}

void MainWindow::connect_buttons()
{
    QObject::connect(ui->solveAStarPathBtn, SIGNAL(pressed()), this, SLOT(on_astarSolve()));
    QObject::connect(ui->setRobotBtn, SIGNAL(pressed()), this, SLOT(on_setRobot()));
    QObject::connect(ui->followPathBtn, SIGNAL(pressed()), this, SLOT(on_start()));
    QObject::connect(ui->pauseSimBtn, SIGNAL(pressed()), this, SLOT(on_pause()));
    QObject::connect(ui->resetSimBtn, SIGNAL(pressed()), this, SLOT(on_reset()));
    QObject::connect(ui->setRandomGoalBtn, SIGNAL(pressed()), this, SLOT(on_setNewGoalLocation()));
    QObject::connect(ui->updateEnvironmentBtn, SIGNAL(pressed()), this, SLOT(on_updateEnvironment()));
    QObject::connect(ui->generateObstaclesBtn, SIGNAL(pressed()), this, SLOT(on_generateObstacles()));

}

void MainWindow::on_timer()
{
    std::array<double,2> waypoint = path[0];
    if (my_math::are_vectors_equal(waypoint, robot.get_loc(), 1.0))
    {
        waypoint = path[0];
        path.erase(path.begin());
    }
    std::array<double,3> kpkdMaxLinear{this->ui->linearKpTextEdit->toPlainText().toDouble(), this->ui->linearKdTextEdit->toPlainText().toDouble(), this->ui->linearMaxVelocityTextEdit->toPlainText().toDouble()};
    std::array<double,3> kpkdMaxAngular{this->ui->angularKpTextEdit->toPlainText().toDouble(), this->ui->angularKdTextEdit->toPlainText().toDouble(), this->ui->angularMaxVelocityTextEdit->toPlainText().toDouble()};
    std::array<double,2> command = pfoll::get_velocity_commands(waypoint, robot.get_pose(),
                                                                 kpkdMaxLinear,
                                                                 kpkdMaxAngular,
                                                                 1/CONTROL_HZ);
    for (int i{0}; i<(SIM_HZ / CONTROL_HZ); i++)
    {
        robot.update_no_noise(command[0], command[1], 1/SIM_HZ);
    }
    if (path.empty())  timer->stop();
    draw::update_triangle(robotGraphicsItem, robot.get_pose());
}

void MainWindow::on_astarSolve()
{
    QElapsedTimer astar_timer;
    astar_timer.start();
    path = astar::astar(env.get_map(),
                        robot.get_loc(),
                        env.get_goalLocation(),
                        ui->maxIterTextEdit->toPlainText().toInt(),
                        obsTolerance);
    qint64 elapsed = astar_timer.elapsed();
    ui->aStarSolveTimeLabel->setText(QString::number(elapsed));
    path = astar::get_sparse_path(path, pathSparsity);
    if (ui->plotAStarCheckBox->isChecked())
    {
        draw::plot_path(path, scene);
    }
}

void MainWindow::on_setRobot()
{
    robot.set_x(ui->xPosTextEdit->toPlainText().toDouble());
    robot.set_y(ui->yPosTextEdit->toPlainText().toDouble());
    robot.set_theta(my_math::deg2rad(ui->thetaTextEdit->toPlainText().toDouble()));

    if (robot.get_x() > env.get_xSize() || robot.get_x() < 0)
    {
        robot.set_x(defaultBotX);
        QMessageBox::information(this, "WARNING", QString("Robot x location must be between 0 and %1").arg(env.get_xSize()));
    }
    if (robot.get_y() > env.get_ySize() || robot.get_y() < 0)
    {
        robot.set_y(defaultBotY);
        QMessageBox::information(this, "WARNING", QString("Robot y location must be between 0 and %1").arg(env.get_ySize()));
    }
    create_scene();
}

void MainWindow::on_start()
{
    if (path.empty())
    {
        QMessageBox::information(this, "WARNING", "No A* Path! Click Solve For A* Path before starting the simulation.");
    }
    else
    {
        timer->start();
    }
}

void MainWindow::on_pause()
{
    timer->stop();
}

void MainWindow::on_reset()
{
    timer->stop();
    path = {};
    obstacles = {};
    robot.set_x(defaultBotX);
    robot.set_y(defaultBotY);
    robot.set_theta(defaultBotTheta);
    env.reset_map();
    ui->aStarSolveTimeLabel->setText("0.0");
    ui->maxIterTextEdit->setPlainText("10000");
    create_scene();
}

void MainWindow::on_setNewGoalLocation()
{
    env.set_goalLocation(my_math::generate_random_point(env.get_xSize(), env.get_ySize()));
    draw::update_star(goalLocationGraphicsItem, env.get_goalLocation());
    set_dependent_text();
}

void MainWindow::on_updateEnvironment()
{
    env.set_goalLocation({this->ui->goalXTextEdit->toPlainText().toDouble(), this->ui->goalYTextEdit->toPlainText().toDouble()});
    if (env.get_goalLocation()[0] > env.get_xSize() || env.get_goalLocation()[0] < 0 || env.get_goalLocation()[1] > env.get_ySize() || env.get_goalLocation()[1] < 0)
    {
        env.set_goalLocation(my_math::generate_random_point(env.get_xSize(), env.get_ySize()));
        QMessageBox::information(this, "WARNING", QString("Goal location must be within the bounds (0 < x < %1 and 0 < y < %2)").arg(env.get_xSize()).arg(env.get_ySize()));
    }
    create_scene();
}

void MainWindow::on_generateObstacles()
{
    const int numObstacles = ui->numObstaclesTextEdit->toPlainText().toInt();
    obstacles = obs::generate_obstacles(env, robot.get_loc(), numObstacles, obstacleSize);
    create_scene();
}

