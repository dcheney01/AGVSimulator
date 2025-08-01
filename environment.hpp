#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <array>

#include "math_utils.hpp"

class Environment
{
public:
    Environment();
    Environment(const int& xSize, const int& ySize);

    const int get_xSize() const;
    const int get_ySize() const;
    my_math::Grid& get_map();
    const std::array<double, 2> get_goalLocation() const;

    void set_goalLocation(std::array<double,2> goalLocation);
    void reset_map();

protected:
    int xSize{0};
    int ySize{0};
    my_math::Grid map;
    std::array<double, 2> goalLocation{0.0, 0.0};
};

#endif // ENVIRONMENT_H
