#ifndef PATH_FOLLOWER_HPP
#define PATH_FOLLOWER_HPP

#include <array>

namespace pfoll {
std::array<double,2> get_velocity_commands(const std::array<double,2>& goalLocation, const std::array<double, 3>& pose, const std::array<double,3>& kpkdMaxLinear, const std::array<double,3>& kpkdMaxAngular, const double& dt);
double pid_control(const double& error, const std::array<double,3>& kpkdMax, const double& dt);
};

#endif // PATH_FOLLOWER_HPP
