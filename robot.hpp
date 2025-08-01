#ifndef ROBOT_H
#define ROBOT_H

#include <array>

class Robot
{
public:
    Robot();
    Robot(const double& x, const double& y, const double& theta);

    void update_no_noise(const double& v, const double& w, const double& dt);

    const double get_x() const;
    const double get_y() const;
    const double get_theta() const;
    const std::array<double,3> get_pose() const;
    const std::array<double,2> get_loc() const;

    void set_x(const double& x);
    void set_y(const double& y);
    void set_theta(const double& theta);

protected:
    std::array<double, 3> pose{0.0, 0.0, 0.0};
};

#endif
