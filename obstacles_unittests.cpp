#include "gtest/gtest.h"

#include "obstacle_utils.hpp"
#include "environment.hpp"

TEST(CollisionCheck, GivenCollision_WhenCheckingCollision_ExpectTrue) {
    std::array<double, 2> obsLoc{1, 1};
    std::array<double, 2> botLoc{1.0, 1.0};
    const int obsSize{1};

    bool result = obs::check_if_collision(obsLoc, botLoc, obsSize);

    ASSERT_TRUE(result);
}

TEST(CollisionCheck, GivenNoCollision_WhenCheckingCollision_ExpectFalse) {
    std::array<double, 2> obsLoc{1, 1};
    std::array<double, 2> botLoc{3.0, 3.0};
    const int obsSize{1};

    bool result = obs::check_if_collision(obsLoc, botLoc, obsSize);

    ASSERT_FALSE(result);
}

TEST(CollisionCheck, GivenCollisionOutsideOfCenter_WhenCheckingCollision_ExpectTrue) {
    std::array<double, 2> obsLoc{1, 1};
    std::array<double, 2> botLoc{3.0, 3.0};
    const int obsSize{3};

    bool result = obs::check_if_collision(obsLoc, botLoc, obsSize);

    ASSERT_TRUE(result);
}

TEST(MarkObstacleOnMap, GivenCenterObstacle_WhenMarkingObstacle_ExpectCenterCellMarked) {
    my_math::Grid map(5, std::vector<double>(5, 0));
    std::array<double, 2> obs{2, 2};
    const int obsSize{1};

    obs::mark_obstacle_on_map(map, obs, obsSize);

    EXPECT_EQ(map[2][2], 1);
    EXPECT_EQ(map[1][1], 0);
    EXPECT_EQ(map[1][2], 1);
    EXPECT_EQ(map[1][3], 0);
    EXPECT_EQ(map[2][3], 1);
    EXPECT_EQ(map[2][1], 1);
    EXPECT_EQ(map[3][1], 0);
    EXPECT_EQ(map[3][2], 1);
    EXPECT_EQ(map[3][3], 0);
}

TEST(MarkObstacleOnMap, GivenBorderObstacle_WhenMarkingObstacle_ExpectTopLeftAndDiagonalCellsMarked) {
    my_math::Grid map(5, std::vector<double>(5, 0));
    std::array<double, 2> obs{0, 0};
    const int obsSize{1};

    obs::mark_obstacle_on_map(map, obs, obsSize);

    EXPECT_EQ(map[0][0], 1);
    EXPECT_EQ(map[1][0], 1);
    EXPECT_EQ(map[0][1], 1);
    EXPECT_EQ(map[1][1], 0);
}

TEST(MarkObstacleOnMap, GivenLargeRadiusObstacle_WhenMarkingObstacle_ExpectCenterAndSurroundingCellsMarked) {
    my_math::Grid map(5, std::vector<double>(5, 0));
    std::array<double, 2> obs{2, 2};
    const int obsSize{2};

    obs::mark_obstacle_on_map(map, obs, obsSize);

    EXPECT_EQ(map[2][2], 1);
    EXPECT_EQ(map[1][2], 1);
    EXPECT_EQ(map[3][2], 1);
    EXPECT_EQ(map[2][1], 1);
    EXPECT_EQ(map[2][3], 1);
}

TEST(GenerateObstacles, GivenValidEnvironmentAndRobot_WhenGeneratingObstacles_ExpectCorrectObstacles) {
    Environment env{5, 5};
    const int numObstacles{3};
    const int obstacleSize{1};

    obs::generate_obstacles(env, {2.0, 2.0}, numObstacles, obstacleSize);

    double sum{0};
    for (int i{0}; i<5; i++) {
        for (int j{0}; j<5; j++) {
            sum += env.get_map()[i][j];
        }
    }

    ASSERT_GE(sum, 3.0);

    EXPECT_EQ(env.get_map()[2][2], 0);
    EXPECT_EQ(env.get_map()[static_cast<int>(env.get_goalLocation()[0])][static_cast<int>(env.get_goalLocation()[1])], 0);
}

TEST(GenerateObstacles, GivenZeroObstacles_WhenGeneratingObstacles_ExpectNoObstacles) {
    Environment env{5, 5};
    const int numObstacles{0};
    const int obstacleSize{1};

    obs::generate_obstacles(env, {2.0, 2.0}, numObstacles, obstacleSize);

    for (int i{0}; i<5; i++) {
        for (int j{0}; j<5; j++) {
            EXPECT_EQ(env.get_map()[i][j], 0);
        }
    }
}

TEST(GenerateObstacles, GivenLargeObstacleSize_WhenGeneratingObstacles_ExpectValidObstacles) {
    Environment env{10, 10};
    const int numObstacles{2};
    const int obstacleSize{3};

    obs::generate_obstacles(env, {2.0, 2.0}, numObstacles, obstacleSize);

    EXPECT_EQ(env.get_map()[2][2], 0);
    EXPECT_EQ(env.get_map()[static_cast<int>(env.get_goalLocation()[0])][static_cast<int>(env.get_goalLocation()[1])], 0);

    double sum{0};
    for (int i{0}; i<10; i++) {
        for (int j{0}; j<10; j++) {
            sum += env.get_map()[i][j];
        }
    }
    ASSERT_GE(sum, 2.0);
}

TEST(GenerateObstacles, GivenSmallEnvironment_WhenGeneratingObstacles_ExpectNoObstacles) {
    Environment env{1, 1};
    const int numObstacles{2};
    const int obstacleSize{1};

    obs::generate_obstacles(env, {0.0, 0.0}, numObstacles, obstacleSize);

    EXPECT_EQ(env.get_map()[0][0], 0);
    EXPECT_EQ(env.get_map()[static_cast<int>(env.get_goalLocation()[0])][static_cast<int>(env.get_goalLocation()[1])], 0);

}

TEST(GenerateObstacles, GivenHugeEnvironment_WhenGeneratingManyObstacles_ExpectObstacles) {
    Environment env{10000, 10000};
    const int numObstacles{10000};
    const int obstacleSize{0};

    obs::generate_obstacles(env, {1.0, 1.0}, numObstacles, obstacleSize);

    double sum{0};
    for (int i{0}; i<10000; i++) {
        for (int j{0}; j<10000; j++) {
            sum += env.get_map()[i][j];
        }
    }
    ASSERT_GE(sum, 9990.0);

    EXPECT_EQ(env.get_map()[1][1], 0);
    EXPECT_EQ(env.get_map()[static_cast<int>(env.get_goalLocation()[0])][static_cast<int>(env.get_goalLocation()[1])], 0);
}
