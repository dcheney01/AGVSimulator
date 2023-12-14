#include "gtest/gtest.h"
#include "path_follower.hpp"
#include "robot.hpp"
#include "math_utils.hpp"

TEST(PIDControlTest, GivenPositiveDistanceError_WhenCommandCalculated_ExpectCorrectResult)
{
    double distanceError{2.0};
    std::array<double, 3> kpkdMax{0.5, 0.1, 5.0};
    double dt{0.1};

    double result = pfoll::pid_control(distanceError, kpkdMax, dt);

    EXPECT_NEAR(result, -1.0, 1e-3);
}

TEST(PIDControlTest, GivenNegativeDistanceError_WhenCommandCalculated_ExpectCorrectResult)
{
    double distanceError {-3.0};
    std::array<double, 3> kpkdMax{0.2, 0.1, 4.0};
    double dt{0.1};

    double result = pfoll::pid_control(distanceError, kpkdMax, dt);

    EXPECT_NEAR(result, 2.4, 1e-3);
}

TEST(PIDControlTest, GivenSaturatedResult_WhenCommandCalculated_ExpectMaxSpeed)
{
    double distanceError{6.0};
    std::array<double, 3> kpkdMax{1.2, 0.05, 3.0};
    double dt{0.1};

    double result = pfoll::pid_control(distanceError, kpkdMax, dt);

    EXPECT_NEAR(result, 3.0, 1e-3);
}


class VelocityCommandsTestFixture : public ::testing::Test {
protected:
    std::array<double, 3> bot{0.0, 0.0, 0.0};
    std::array<double, 2> goalLocation{10, 10};
    std::array<double, 2> zerosLocation{0, 0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.0, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.0, my_math::PI};
    double dt{0.1};
    std::array<double, 2> commandResults;
    double tol{1e-4};
};

TEST_F(VelocityCommandsTestFixture, GivenDefaultBotGoalLocation_WhenVelocityCommandsChecked_ExpectNonzeroCommands)
{
    commandResults = pfoll::get_velocity_commands(goalLocation, bot, kpkdMaxLinear, kpkdMaxAngular, dt);

    ASSERT_NEAR(commandResults[0], 7.50, tol);
    ASSERT_NEAR(commandResults[1], my_math::PI / 4, tol);
}

TEST_F(VelocityCommandsTestFixture, GivenBotAtGoalLocation_WhenVelocityCommandsChecked_ExpectZeroCommands)
{
    commandResults = pfoll::get_velocity_commands(zerosLocation, bot, kpkdMaxLinear, kpkdMaxAngular, dt);

    ASSERT_NEAR(commandResults[0], 0.0, tol);
    ASSERT_NEAR(commandResults[1], 0.0, tol);
}

TEST_F(VelocityCommandsTestFixture, GivenBotFacingGoalLocation_WhenVelocityCommandsChecked_ExpectNonzeroLinearCommand)
{
    bot[2] = my_math::calculate_angle_between_pose_and_goal(bot, goalLocation);
    commandResults = pfoll::get_velocity_commands(goalLocation, bot, kpkdMaxLinear, kpkdMaxAngular, dt);

    ASSERT_NEAR(commandResults[0], 10.0, tol);
    ASSERT_NEAR(commandResults[1], 0.0, tol);
}

TEST(ReachWaypointTest, GivenMaxAngleError_WhenForwardSimulated_ExpectZeroLinearVelocityCommand)
{
    Robot bot;
    std::array<double, 2> goal{-1, 0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.1, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.0, my_math::PI};
    double dt{0.1};

    std::array<double, 2> command = pfoll::get_velocity_commands(goal, bot.get_pose(), kpkdMaxLinear, kpkdMaxAngular, dt);

    ASSERT_NEAR(command[0], 0.0, 1e-1);
    ASSERT_NEAR(command[1], my_math::PI, 1e-1);
}

TEST(ReachWaypointTest, GivenRobotAtWaypoint_WhenForwardSimulated_ExpectRobotToStayAtWaypoint)
{
    Robot bot;
    std::array<double, 2> goal{0, 0};
    int iter{0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.1, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.1, my_math::PI};
    double dt{0.01};

    while (iter < 10)
    {
        std::array<double, 2> command = pfoll::get_velocity_commands(goal, bot.get_pose(), kpkdMaxLinear, kpkdMaxAngular, dt);
        for (int i{0}; i < 10; i++)
        {
            bot.update_no_noise(command[0], command[1], 0.001);
        }
        iter++;
    }

    ASSERT_NEAR(bot.get_x(), goal[0], 1e-1);
    ASSERT_NEAR(bot.get_y(), goal[1], 1e-1);
}

TEST(ReachWaypointTest, GivenSingleWaypointStraightLineAwayInX_WhenPathFollowingUsed_ExpectRobotToArriveAtWaypoint)
{
    Robot bot;
    std::array<double, 2> goal{4, 4};
    int iter{0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.0, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.0, my_math::PI};
    double dt{0.01};

    while (iter < 100)
    {
        std::array<double, 2> command = pfoll::get_velocity_commands(goal, bot.get_pose(), kpkdMaxLinear, kpkdMaxAngular, dt);

        for (int i{0}; i < 10; i++)
        {
            bot.update_no_noise(command[0], command[1], 0.01);
        }
        iter++;
    }

    ASSERT_NEAR(bot.get_x(), goal[0], 1e-1);
    ASSERT_NEAR(bot.get_y(), goal[1], 1e-1);
}

TEST(ReachWaypointTest, GivenSingleWaypointStraightLineAwayInY_WhenPathFollowingUsed_ExpectRobotToArriveAtWaypoint)
{
    Robot bot;
    std::array<double, 2> goal{0, 4};
    int iter{0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.0, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.0, my_math::PI};
    double dt{0.01};

    while (iter < 1000)
    {
        std::array<double, 2> command = pfoll::get_velocity_commands(goal, bot.get_pose(), kpkdMaxLinear, kpkdMaxAngular, dt);

        for (int i{0}; i < 10; i++)
        {
            bot.update_no_noise(command[0], command[1], 0.01);
        }
        iter++;
    }

    ASSERT_NEAR(bot.get_x(), goal[0], 1e-1);
    ASSERT_NEAR(bot.get_y(), goal[1], 1e-1);
}

TEST(ReachWaypointTest, GivenWaypointFarAway_WhenPathFollowingUsed_ExpectRobotToArriveAtWaypoint)
{
    Robot bot;
    std::array<double, 2> goal{200, 400};
    int iter{0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.0, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.0, my_math::PI};
    double dt{0.05};

    while (iter < 1000)
    {
        std::array<double, 2> command = pfoll::get_velocity_commands(goal, bot.get_pose(), kpkdMaxLinear, kpkdMaxAngular, dt);

        for (int i{0}; i < 10; i++)
        {
            bot.update_no_noise(command[0], command[1], 0.05);
        }
        iter++;
    }

    ASSERT_NEAR(bot.get_x(), goal[0], 1e-1);
    ASSERT_NEAR(bot.get_y(), goal[1], 1e-1);
}

TEST(ReachWaypointTest, GivenAnotherWaypointFarAway_WhenPathFollowingUsed_ExpectRobotToArriveAtWaypoint)
{
    Robot bot;
    std::array<double, 2> goal{600, 700};
    int iter{0};
    std::array<double, 3> kpkdMaxLinear{1.0, 0.0, 10.0};
    std::array<double, 3> kpkdMaxAngular{1.0, 0.0, my_math::PI};
    double dt{0.05};

    while (iter < 1000)
    {
        std::array<double, 2> command = pfoll::get_velocity_commands(goal, bot.get_pose(), kpkdMaxLinear, kpkdMaxAngular, 10*dt);

        for (int i{0}; i < 10; i++)
        {
            bot.update_no_noise(command[0], command[1], 0.05);
        }
        iter++;
    }

    ASSERT_NEAR(bot.get_x(), goal[0], 1e-1);
    ASSERT_NEAR(bot.get_y(), goal[1], 1e-1);
}
