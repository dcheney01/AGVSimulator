#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <array>
#include <vector>

namespace my_math {
    typedef std::vector<std::vector<double>> Grid;
    const double PI{3.14159265358979311600};
    const double RAD_2_DEG{180/PI};

    double wrap_angle_NegPI_PI(const double& angle);
    double rad2deg(const double& angleRads);
    double deg2rad(const double& angleDegs);
    std::array<double, 2> generate_random_point(const int& xSize, const int& ySize);
    std::array<double,2> find_polygon_centroid(const std::vector<std::array<double,2>>& points);
    double calculate_xy_distance(const std::array<double, 2>& v1, const std::array<double, 2>& v2);
    double calculate_angle_between_pose_and_goal(const std::array<double, 3>& pose, const std::array<double, 2>& goal);
    bool are_vectors_equal(const std::array<double,2>& v1, const std::array<double,2>& v2, double tol=1);
    int get_sign(const double& num);
};

#endif // MATH_UTILS_HPP
