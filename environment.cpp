#include "environment.hpp"
#include <random>

Environment::Environment()
    : xSize{0},
    ySize{0},
    map{my_math::Grid(xSize, std::vector<double>(ySize, 0))},
    goalLocation{{0,0}}
{
}

Environment::Environment(const int& _xSize, const int& _ySize)
    : xSize{_xSize}, ySize{_ySize}, map{my_math::Grid(_xSize, std::vector<double>(_ySize, 0))}
{
    goalLocation = my_math::generate_random_point(xSize, ySize);
}

const int Environment::get_xSize() const
{
    return xSize;
}

const int Environment::get_ySize() const
{
    return ySize;
}

my_math::Grid& Environment::get_map()
{
    return map;
};

const std::array<double,2> Environment::get_goalLocation() const
{
    return goalLocation;
}

void Environment::reset_map()
{
    for (int i{0}; i < xSize; i++) {
        for (int j{0}; j < ySize; j++) {
            map[i][j] = 0;
        }
    }
}

void Environment::set_goalLocation(std::array<double,2> goalLocation)
{
    this->goalLocation = goalLocation;
}
