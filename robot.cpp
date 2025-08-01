#include "robot.hpp"

#include "math_utils.hpp"

Robot::Robot(){}

Robot::Robot(const double &x, const double &y, const double &theta)
{
    pose[0] = x;
    pose[1] = y;
    pose[2] = theta;
}

void Robot::update_no_noise(const double& v, const double& w, const double& dt)
{
    double deltaX{0.0};
    double deltaY{0.0};

    if (w == 0.0)
    {
        deltaX = v * cos(pose[2])*dt;
        deltaY = v * sin(pose[2])*dt;
    }
    else
    {
        deltaX = (v / w) * (sin(pose[2] + w * dt) - sin(pose[2]));
        deltaY = (v / w) * (-cos(pose[2] + w * dt) + cos(pose[2]));
    }

    pose[0] += deltaX;
    pose[1] += deltaY;
    pose[2] = my_math::wrap_angle_NegPI_PI(pose[2] + w*dt);
}

const double Robot::get_x() const
{
    return pose[0];
}

const double Robot::get_y() const
{
    return pose[1];
}

const double Robot::get_theta() const
{
    return pose[2];
}

const std::array<double,3> Robot::get_pose() const
{
    return pose;
}

const std::array<double,2> Robot::get_loc() const
{
    return {pose[0], pose[1]};
}

void Robot::set_x(const double& x)
{
    pose[0] = x;
}
void Robot::set_y(const double& y)
{
    pose[1] = y;
}
void Robot::set_theta(const double& theta)
{
    pose[2] = theta;
}
