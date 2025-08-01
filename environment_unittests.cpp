#include "gtest/gtest.h"
#include "environment.hpp"
#include "obstacle_utils.hpp"

TEST(DefaultEnvironment, GivenEmptyInitializer_WhenClassMembersChecked_ExpectZeros)
{
    Environment env;
    std::vector<std::vector<double>> defaultMap = env.get_map();

    EXPECT_EQ(env.get_xSize(), 0);
    EXPECT_EQ(env.get_ySize(), 0);
    EXPECT_TRUE(defaultMap.empty());
}

TEST(DefaultEnvironment, GivenEmpyEnvironment_WhenGoalSet_ExpectGoalSetCorrectly)
{
    Environment env{1000, 1000};
    env.set_goalLocation({355, 768});

    EXPECT_EQ(env.get_goalLocation()[0], 355);
    EXPECT_EQ(env.get_goalLocation()[1], 768);
}

class DefaultEnvironmentTestFixture: public ::testing::Test
{
protected:
    Environment env{500, 600};
    const double tol{1e-5};
};

TEST_F(DefaultEnvironmentTestFixture, GivenInitializedEnv_WhenSizeAndGoalLocationChecked_ExpectGivenValuesAndNewGoalLocation)
{
    ASSERT_NEAR(env.get_xSize(), 500, tol);
    ASSERT_NEAR(env.get_ySize(), 600, tol);

    EXPECT_GE(env.get_goalLocation()[0], 0);
    EXPECT_LE(env.get_goalLocation()[0], env.get_xSize());
    EXPECT_GE(env.get_goalLocation()[1], 0);
    EXPECT_LE(env.get_goalLocation()[1], env.get_ySize());
}

TEST(EnvironmentMapTest, GivenEnvironment_WhenGetMapCalled_ExpectCorrectlySizedMap)
{
    my_math::Grid testMap{
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    };

    Environment env{5, 4};

    ASSERT_EQ(env.get_map(), testMap);
    ASSERT_EQ(env.get_map().size(), 5);
    ASSERT_EQ(env.get_map()[0].size(), 4);
    ASSERT_EQ(env.get_map()[0][0], 0);
}

TEST(EnvironmentResetTest, GivenEnvironmentWIthObstacles_WhenResetCalled_ExpectAllZeros)
{
    Environment env{5, 4};
    obs::generate_obstacles(env, {0,0}, 10, 1);

    env.reset_map();

    for (int i{0}; i < env.get_xSize(); i++) {
        for (int j{0}; j < env.get_ySize(); j++) {
            ASSERT_EQ(env.get_map()[0][0], 0);
        }
    }
}
