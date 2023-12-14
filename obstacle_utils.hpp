#ifndef OBSTACLE_UTILS_HPP
#define OBSTACLE_UTILS_HPP

#include <vector>

#include "environment.hpp"
#include "math_utils.hpp"

namespace obs
{
bool check_if_collision(const std::array<double,2>& obsLoc, const std::array<double,2>& loc, const int& obsSize);
void mark_obstacle_on_map(my_math::Grid& map, const std::array<double,2>& obs, const int& obsSize);
std::vector<std::array<double,2>> generate_obstacles(Environment& env, const std::array<double,2>& botLoc, const int& numObstacles, const int& obstacleSize);
}

#endif // OBSTACLE_UTILS_HPP
