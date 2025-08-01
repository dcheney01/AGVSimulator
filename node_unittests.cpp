#include "gtest/gtest.h"
#include "node.hpp"


class NodeTestFixture: public ::testing::Test
{

protected:
    double x{100};
    double y{200};
    double costFromStart{5};
    double costToGoal{14};
    double tol{1e-4};

    std::array<double, 2> loc{x, y};
    Node n{loc, costFromStart, costToGoal, nullptr};
};

TEST_F(NodeTestFixture, GivenInitializedNode_WhenDataChecked_ExpectSetValues)
{
    ASSERT_NEAR(n.get_location()[0], x, tol);
    ASSERT_NEAR(n.get_location()[1], y, tol);
    ASSERT_NEAR(n.get_costFromStart(), costFromStart, tol);
    ASSERT_NEAR(n.get_costToGoal(), costToGoal, tol);
    ASSERT_NEAR(n.get_totalCost(), costFromStart + costToGoal, tol);
    ASSERT_TRUE(n.get_parent() == nullptr);
}

TEST_F(NodeTestFixture, GivenInitializedNode_WhenLocationSet_ExpectSetValue)
{
    std::array<double, 2> newLoc{10, 20};

    n.set_location(newLoc);

    ASSERT_EQ(n.get_location(), newLoc);
}

TEST_F(NodeTestFixture, GivenInitializedNode_WhenCostFromStartSet_ExpectSetValue)
{
    double newCostFromStart{15};
    n.set_costFromStart(newCostFromStart);

    ASSERT_EQ(n.get_costFromStart(), newCostFromStart);
}

TEST_F(NodeTestFixture, GivenInitializedNode_WhenCostToGoalSet_ExpectSetValue)
{
    double newCostToGoal{25};
    n.set_costToGoal(newCostToGoal);

    ASSERT_EQ(n.get_costToGoal(), newCostToGoal);
}

TEST_F(NodeTestFixture, GivenInitializedNode_WhenParenSet_ExpectSetValue)
{
    std::shared_ptr<Node> newParent = std::make_shared<Node>(loc, costFromStart, costToGoal, nullptr);
    n.set_parent(newParent);

    ASSERT_EQ(n.get_parent(), newParent);
}

TEST(NodeEqualityTest, GivenEqualNodes_WhenEqualityChecked_ExpectEquals) {
    Node node1({1, 2}, 3, 4, nullptr);
    Node node2({1, 2}, 3, 4, nullptr);

    ASSERT_EQ(node1, node2);
}

TEST(NodeEqualityTest, GivenDifferentLocationsNodes_WhenEqualityChecked_ExpectNotEquals) {
    Node node1({1, 2}, 3, 4, nullptr);
    Node node2({3, 4}, 3, 4, nullptr);

    ASSERT_NE(node1, node2);
}

TEST(NodeEqualityTest, GivenDifferentCostsAndParentNodes_WhenEqualityChecked_ExpectNotEquals) {
    Node node1({1, 2}, 3, 4, nullptr);
    Node node2({1, 2}, 5, 6, nullptr);

    ASSERT_NE(node1, node2);
}

TEST(NodeInequalityTest, GivenUnequalNodes_WhenInEqualityChecked_ExpectNotEquals) {
    Node node1({1, 2}, 3, 4, nullptr);
    Node node2({3, 4}, 5, 6, nullptr);

    ASSERT_NE(node1, node2);
}

TEST(NodeInequalityTest, GivenEqualNodes_WhenInequalityChecked_ExpectFalse) {
    Node node1({1, 2}, 3, 4, nullptr);
    Node node2({1, 2}, 3, 4, nullptr);

    ASSERT_FALSE(node1 != node2);
}

TEST(NodeInequalityTest, GivenDifferentParentsNodes_WhenInequalityChecked_ExpectNE) {
    Node node1({1, 2}, 3, 4, nullptr);
    Node node2({1, 2}, 3, 4, std::make_shared<Node>(node1));

    ASSERT_NE(node1, node2);
}
