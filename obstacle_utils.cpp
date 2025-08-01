#include "obstacle_utils.hpp"

namespace obs
{

bool check_if_collision(const std::array<double,2>& obsLoc, const std::array<double,2>& loc, const int& obsSize)
{
    return my_math::calculate_xy_distance(obsLoc, loc) <= obsSize;
}

void mark_obstacle_on_map(my_math::Grid& map, const std::array<double,2>& obs, const int& obsSize)
{
    const int centerX{static_cast<int>(obs[0])};
    const int centerY{static_cast<int>(obs[1])};

    map[centerX][centerY] = 1;

    for (int i{centerX - obsSize}; i <= centerX + obsSize; i++) {
        for (int j{centerY - obsSize}; j <= centerY + obsSize; j++) {
            if (i >= 0 && i < map.size() && j >= 0 && j < map[0].size()) {
                if (sqrt(pow(i - centerX, 2) + pow(j - centerY, 2)) <= obsSize) {
                    map[i][j] = 1;
                }
            }
        }
    }
}

std::vector<std::array<double,2>> generate_obstacles(Environment& env, const std::array<double,2>& botLoc, const int& numObstacles, const int& obstacleSize)
{
    const int maxIter{100};
    std::array<double,2> goalLoc{env.get_goalLocation()};
    std::array<double,2> newObstacle{0, 0};
    std::vector<std::array<double,2>> obstacles{};

    int numGeneratedObstacles{0};

    while (numGeneratedObstacles < numObstacles)
    {
        int iter{0};
        do {
            newObstacle = my_math::generate_random_point(env.get_xSize(), env.get_ySize());
            iter++;
        } while ((check_if_collision(newObstacle, botLoc, obstacleSize) ||
                  check_if_collision(newObstacle, goalLoc, obstacleSize)) &&
                 iter < maxIter);

        if (iter < maxIter)
        {
            mark_obstacle_on_map(env.get_map(), newObstacle, obstacleSize);
            obstacles.push_back(newObstacle);
        }
        numGeneratedObstacles++;
    }
    return obstacles;
}

}
