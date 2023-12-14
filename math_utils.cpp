#include "math_utils.hpp"
#include <numeric>
#include <random>

namespace my_math
{
double wrap_angle_NegPI_PI(const double& angle)
{
    double result = angle - 2*PI * floor((angle+PI) / (2*PI));
    if (result <= -PI)
    {
        result += 2*PI;
    }
    return result;
}

double rad2deg(const double& angleRads)
{
    return angleRads*RAD_2_DEG;
}

double deg2rad(const double& angleDegs)
{
    return angleDegs / RAD_2_DEG;
}


std::array<double, 2> generate_random_point(const int& xSize, const int& ySize)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> disX(0, xSize-1);
    std::uniform_int_distribution<int> disY(0, ySize-1);

    int randomX = disX(gen);
    int randomY = disY(gen);

    return {static_cast<double>(randomX), static_cast<double>(randomY)};
}


std::array<double,2> find_polygon_centroid(const std::vector<std::array<double,2>>& points)
{
    double sumX{0.0};
    double sumY{0.0};
    for (int i{0}; i < points.size(); i++)
    {
        sumX += points[i][0];
        sumY += points[i][1];
    }
    sumX /= points.size();
    sumY /= points.size();

    return {sumX, sumY};
}

double calculate_xy_distance(const std::array<double, 2>& v1, const std::array<double, 2>& v2)
{
    return sqrt(pow(v1[0] - v2[0], 2) + pow(v1[1] - v2[1], 2));
}

double calculate_angle_between_pose_and_goal(const std::array<double, 3>& pose, const std::array<double, 2>& goal)
{
    std::array<double,2> goalVector = {goal[0] - pose[0], goal[1] - pose[1]};
    double angle = atan2(goalVector[1], goalVector[0]) - pose[2];
    return wrap_angle_NegPI_PI(angle);
}

bool are_vectors_equal(const std::array<double,2>& v1, const std::array<double,2>& v2, double tol)
{
    return calculate_xy_distance(v1, v2) < tol;
}

int get_sign(const double& num)
{
    if (num < 0)
    {
        return -1;
    }
    return 1;
}

};
