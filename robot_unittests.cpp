#include "gtest/gtest.h"
#include "robot.hpp"

TEST(Robot, GivenInitializedPose_WhenPoseChecked_ExpectGivenValues)
{
    Robot bot{3.0, 6.0, 0.5};
    ASSERT_NEAR(bot.get_x(), 3.0, 1e-3);
    ASSERT_NEAR(bot.get_y(), 6.0, 1e-3);
    ASSERT_NEAR(bot.get_theta(), 0.5, 1e-3);
}

class DefaultRobotTestFixture: public ::testing::Test
{
protected:
    Robot defaultBot;
    const double newX{200.0};
    const double newY{150.0};
    const double newTheta{1.50};
    const double tol{1e-6};
};

TEST_F(DefaultRobotTestFixture, GivenDefaultRobotInitialization_WhenPoseChecked_ExpectDefaults)
{
    std::array<double,3> pose{0.0, 0.0, 0.0};
    std::array<double,2> zeroLoc{0.0, 0.0};

    ASSERT_NEAR(defaultBot.get_x(), 0.0, tol);
    ASSERT_NEAR(defaultBot.get_y(), 0.0, tol);
    ASSERT_NEAR(defaultBot.get_theta(), 0.0, tol);
    ASSERT_EQ(defaultBot.get_pose(), pose);
    ASSERT_EQ(defaultBot.get_loc(), zeroLoc);
}

TEST_F(DefaultRobotTestFixture, GivenRobotWithSetValues_WhenPoseChecked_ExpectSetValues)
{
    defaultBot.set_x(newX);
    defaultBot.set_y(newY);
    defaultBot.set_theta(newTheta);
    std::array<double,3> pose{newX, newY, newTheta};

    ASSERT_NEAR(defaultBot.get_x(), newX, tol);
    ASSERT_NEAR(defaultBot.get_y(), newY, tol);
    ASSERT_NEAR(defaultBot.get_theta(), newTheta, tol);
    ASSERT_EQ(defaultBot.get_pose(), pose);
}

class RobotMotionTestFixture: public ::testing::Test
{
protected:
    Robot bot;
    const double tol{1e-6};
    const double dt{0.5};
};


TEST_F(RobotMotionTestFixture, GivenLinearCommandNoNoise_WhenPoseChecked_ExpectMovementInX)
{
    bot.update_no_noise(1.0, 0.0, dt);

    ASSERT_NEAR(bot.get_x(), 0.5, tol);
    ASSERT_NEAR(bot.get_y(), 0.0, tol);
    ASSERT_NEAR(bot.get_theta(), 0.0, tol);
}

TEST_F(RobotMotionTestFixture, GivenRotateCommandNoNoise_WhenPoseChecked_ExpectThetaUpdated)
{
    bot.update_no_noise(0.0, 1.0, dt);

    ASSERT_NEAR(bot.get_x(), 0.0, tol);
    ASSERT_NEAR(bot.get_y(), 0.0, tol);
    ASSERT_NEAR(bot.get_theta(), 0.5, tol);
}

TEST_F(RobotMotionTestFixture, GivenFullUpdateCommandNoNoise_WhenPoseChecked_ExpectXYThetaUpdated)
{
    bot.update_no_noise(5.0, 1.0, dt);

    ASSERT_NEAR(bot.get_x(), 2.3971276, tol);
    ASSERT_NEAR(bot.get_y(), 0.612087, tol);
    ASSERT_NEAR(bot.get_theta(), 0.5, tol);
}
