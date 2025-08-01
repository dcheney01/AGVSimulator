#include "gtest/gtest.h"
#include "math_utils.hpp"

#include <array>


TEST(AngleConversionTest, GivenAnglesInRads_WhenConvertedToDegrees_ExpectCorrectConversion)
{
    const double PI{3.14159265358979323846};
    double tol{1e-7};

    ASSERT_NEAR(my_math::rad2deg(0.0), 0.0, tol);
    ASSERT_NEAR(my_math::rad2deg(PI / 2.0), 90.0, tol);
    ASSERT_NEAR(my_math::rad2deg(PI), 180.0, tol);
    ASSERT_NEAR(my_math::rad2deg(2.0 * PI), 360.0, tol);
    ASSERT_NEAR(my_math::rad2deg(-PI / 4.0), -45.0, tol);
    ASSERT_NEAR(my_math::rad2deg(-3.0 * PI / 2.0), -270.0, tol);
}

TEST(AngleConversionTest, GivenAnglesInDegs_WhenConvertedToRads_ExpectCorrectConversion)
{
    const double PI{3.14159265358979323846};
    double tol{1e-7};

    ASSERT_NEAR(my_math::deg2rad(0.0), 0.0, tol);
    ASSERT_NEAR(my_math::deg2rad(90.0), PI / 2.0, tol);
    ASSERT_NEAR(my_math::deg2rad(180.0), PI, tol);
    ASSERT_NEAR(my_math::deg2rad(360.0), 2.0 * PI, tol);
    ASSERT_NEAR(my_math::deg2rad(-45.0), -PI / 4.0, tol);
    ASSERT_NEAR(my_math::deg2rad(-270.0), -3.0 * PI / 2.0, tol);
}

TEST(WrapAngleTest, GivenPositiveAngle_WhenWrapped_ExpectWrappedAngle)
{
    double wrappedAngle = my_math::wrap_angle_NegPI_PI(1.5*my_math::PI);

    EXPECT_NEAR(wrappedAngle, -.5*my_math::PI, 1e-4);
}

TEST(WrapAngleTest, GivenZeroAngle_WhenWrapped_ExpectZeroAngle)
{
    double wrappedAngle = my_math::wrap_angle_NegPI_PI(0.0);

    EXPECT_NEAR(wrappedAngle, 0.0, 1e-3);
}

TEST(WrapAngleTest, GivenNegativeAngle_WhenWrapped_ExpectPositiveWrappedAngle)
{
    double wrappedAngle = my_math::wrap_angle_NegPI_PI(-1.2*my_math::PI);

    EXPECT_NEAR(wrappedAngle, 0.8*my_math::PI, 1e-3);
}

TEST(WrapAngleTest, GivenAngleInRange_WhenWrapped_ExpectSameAngle)
{
    double wrappedAngle = my_math::wrap_angle_NegPI_PI(my_math::PI / 2);

    EXPECT_NEAR(wrappedAngle, my_math::PI / 2, 1e-3);
}

TEST(RandomPointGeneration, GivenLmits_WhenGenerated_ExpectPointWithingBounds)
{
    const int xSize{500};
    const int ySize{700};

    for (int i{0}; i < 30; i++)
    {
        std::array<double, 2> randomPoint = my_math::generate_random_point(xSize, ySize);

        ASSERT_GE(randomPoint[0], 0);
        ASSERT_LE(randomPoint[0], xSize-1);
        ASSERT_GE(randomPoint[1], 0);
        ASSERT_LE(randomPoint[1], ySize-1);
    }
}

TEST(CalcXYDistance, GivenVectorsWithSameY_WhenDistanceCalculated_ExpectEuclideanDistance)
{
    std::array<double, 2> v1{100, 100};
    std::array<double, 2> v2{200, 100};

    double distance = my_math::calculate_xy_distance(v1, v2);

    ASSERT_NEAR(distance, 100.0, 1e-5);
}

TEST(CalcXYDistance, GivenVectors_WhenDistanceCalculated_ExpectEuclideanDistance)
{
    std::array<double, 2> v1{0, 0};
    std::array<double, 2> v2{5, 5};

    double distance = my_math::calculate_xy_distance(v1, v2);

    ASSERT_NEAR(distance, 7.0710678, 1e-5);
}

TEST(AreVectorsEqual, GivenEqualVectors_WhenComparedWithTolerance_ExpectTrue)
{
    std::array<double, 2> v1{1.0, 2.0};
    std::array<double, 2> v2{1.0, 2.0};

    bool result = my_math::are_vectors_equal(v1, v2, 1e-5);

    ASSERT_TRUE(result);
}

TEST(AreVectorsEqual, GivenNonEqualVectors_WhenComparedWithTolerance_ExpectFalse)
{
    std::array<double, 2> v1{1.0, 2.0};
    std::array<double, 2> v2{1.1, 1.8};

    bool result = my_math::are_vectors_equal(v1, v2, 1e-5);

    ASSERT_FALSE(result);
}

TEST(AreVectorsEqual, GivenEqualVectors_WhenComparedWithNoTolerance_ExpectTrue)
{
    std::array<double, 2> v1{1.0, 2.0};
    std::array<double, 2> v2{1.0, 2.0};

    bool result = my_math::are_vectors_equal(v1, v2);

    ASSERT_TRUE(result);
}

TEST(AreVectorsEqual, GivenNonEqualVectors_WhenComparedWithNoTolerance_ExpectFalse)
{
    std::array<double, 2> v1{1.0, 8.0};
    std::array<double, 2> v2{1.1, 1.8};

    bool result = my_math::are_vectors_equal(v1, v2);

    ASSERT_FALSE(result);
}

TEST(CalculateAngleBetweenPoseAndGoalTest, GivenZeroAngleDifference_WhenCalculated_ExpectZeroAngle)
{
    std::array<double, 3> pose{0.0, 0.0, 0.0};
    std::array<double, 2> goal{1, 0};

    double result = my_math::calculate_angle_between_pose_and_goal(pose, goal);

    EXPECT_NEAR(result, 0.0, 1e-5);
}

TEST(CalculateAngleBetweenPoseAndGoalTest, GivenPositiveAngleDifference_WhenCalculated_ExpectPositiveAngle)
{
    std::array<double, 3> pose{0.0, 0.0, 0.0};
    std::array<double, 2> goal{-1, -1};

    double result = my_math::calculate_angle_between_pose_and_goal(pose, goal);

    EXPECT_NEAR(result, -my_math::PI*3/4, 1e-5);
}

TEST(CalculateAngleBetweenPoseAndGoalTest, GivenNegativeAngleDifference_WhenCalculated_ExpectNegativeAngle)
{
    std::array<double, 3> pose{0.0, 0.0, 0.0};
    std::array<double, 2> goal{1, 1};

    double result = my_math::calculate_angle_between_pose_and_goal(pose, goal);

    EXPECT_NEAR(result, my_math::PI/4, 1e-5);
}

TEST(CalculateAngleBetweenPoseAndGoalTest, Given180DegreeAngleDifference_WhenCalculated_ExpectPIAngle)
{
    std::array<double, 3> pose{0.0, 0.0, my_math::PI};
    std::array<double, 2> goal{1, 0};

    double result = my_math::calculate_angle_between_pose_and_goal(pose, goal);

    EXPECT_NEAR(result, my_math::PI, 1e-5);
}

TEST(CalculateAngleBetweenPoseAndGoalTest, GivenSmallAngleDifference_WhenCalculated_ExpectSmallAngle)
{
    std::array<double, 3> pose{0.0, 0.0, 0.1};
    std::array<double, 2> goal{1, 1};

    double result = my_math::calculate_angle_between_pose_and_goal(pose, goal);

    EXPECT_NEAR(result, (my_math::PI/4.0 - 0.1), 1e-5);
}

TEST(CalculateAngleBetweenPoseAndGoalTest, GivenLargeAngleDifference_WhenCalculated_ExpectWrappedAngle)
{
    std::array<double, 3> pose{0.0, 0.0, 6.0};
    std::array<double, 2> goal{1, 1};

    double result = my_math::calculate_angle_between_pose_and_goal(pose, goal);

    EXPECT_NEAR(result, 1.0685843, 1e-5);
}

TEST(GetSignTest, GivenPositiveNumber_WhenGetSignCalled_ExpectPositiveSign)
{
    double positiveNumber{42.5};

    int result = my_math::get_sign(positiveNumber);

    EXPECT_EQ(result, 1);
}

TEST(GetSignTest, GivenNegativeNumber_WhenGetSignCalled_ExpectNegativeSign)
{
    double negativeNumber{-17.8};

    int result = my_math::get_sign(negativeNumber);

    EXPECT_EQ(result, -1);
}

TEST(GetSignTest, GivenZero_WhenGetSignCalled_ExpectPositiveSign)
{
    double zeroNumber{0.0};

    int result = my_math::get_sign(zeroNumber);

    EXPECT_EQ(result, 1);
}
