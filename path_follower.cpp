#include "path_follower.hpp"
#include "math_utils.hpp"

namespace pfoll {

std::array<double,2> get_velocity_commands(const std::array<double,2>& goalLocation,
                                            const std::array<double, 3>& pose,
                                            const std::array<double,3>& kpkdMaxLinear,
                                            const std::array<double,3>& kpkdMaxAngular,
                                            const double& dt
                                            )
{
    std::array<double,2> velocityCommands{0.0, 0.0};

    double distanceError = my_math::calculate_xy_distance({pose[0], pose[1]}, goalLocation);
    double angleToGoal = my_math::calculate_angle_between_pose_and_goal(pose, goalLocation);

    velocityCommands[1] = pid_control(angleToGoal, kpkdMaxAngular, dt);
    if (distanceError < std::numeric_limits<double>::epsilon())
    {
        velocityCommands[1] = 0.0;
    }

    double linearScalingFactor{1.0 - (std::abs(angleToGoal) / my_math::PI)};
    velocityCommands[0] = linearScalingFactor * pid_control(distanceError, kpkdMaxLinear, dt);
    return velocityCommands;
}

double pid_control(const double& error, const std::array<double,3>& kpkdMax, const double& dt)
{
    static double previousError = 0.0;

    double proportional{kpkdMax[0] * error};
    double derivative{kpkdMax[1] * ((error - previousError) / dt)};

    double command{proportional - derivative};

    if (command == 0)
    {
        return command;
    }
    if (std::abs(command) > kpkdMax[2])
    {
        command = my_math::get_sign(command)*kpkdMax[2];
    }
    return command;
}
};
