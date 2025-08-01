#include "gtest/gtest.h"
#include "astar.hpp"
#include "node.hpp"
#include "environment.hpp"
#include "obstacle_utils.hpp"

class AstarInsertOpenSetTestFixture: public ::testing::Test
{
protected:
    std::shared_ptr<Node> n1 = std::make_shared<Node>(std::array<double, 2>{100, 100}, 10.0, 20.0, nullptr);
    std::shared_ptr<Node> n2 = std::make_shared<Node>(std::array<double, 2>{120, 100}, 20.0, 20.0, nullptr);
    std::shared_ptr<Node> n3 = std::make_shared<Node>(std::array<double, 2>{200, 100}, 5.0, 15.0, nullptr);
    std::shared_ptr<Node> n4 = std::make_shared<Node>(std::array<double, 2>{150, 100}, 50.0, 20.0, nullptr);
    std::vector<std::shared_ptr<Node>> openSet{n1, n2};
};

TEST_F(AstarInsertOpenSetTestFixture, GivenNodeWithLowestCost_WhenNodeInsertedIntoOpenSet_ExpectNodeToBeFirstInTheOpenSet)
{
    astar::insert_into_openset(openSet, n3);
    EXPECT_TRUE(openSet[0] == n3);
    EXPECT_TRUE(openSet[1] == n1);
    EXPECT_TRUE(openSet[2] == n2);
    EXPECT_TRUE(openSet.size() == 3);
}

TEST_F(AstarInsertOpenSetTestFixture, GivenNodeWithHighestCost_WhenNodeInsertedIntoOpenSet_ExpectNodeToBeLastInTheOpenSet)
{
    astar::insert_into_openset(openSet, n4);
    EXPECT_TRUE(openSet[0] == n1);
    EXPECT_TRUE(openSet[1] == n2);
    EXPECT_TRUE(openSet[2] == n4);
    EXPECT_TRUE(openSet.size() == 3);
}

TEST_F(AstarInsertOpenSetTestFixture, GivenEmptyOpenSet_WhenNodeInsertedIntoOpenSet_ExpectNodeToBeOnlyNodeInOpenSet)
{
    openSet.clear();
    astar::insert_into_openset(openSet, n1);
    EXPECT_TRUE(openSet[0] == n1);
    EXPECT_TRUE(openSet.size() == 1);
}

TEST(AstarCheckNodeLocTest, GivenValidNodeLoc_WhenCheckedIfValidAndNotInClosedSet_ExpectTrue)
{
    my_math::Grid grid(2, std::vector<double>(2, 0));
    std::vector<std::shared_ptr<Node>> closedSet{};
    std::array<double, 2> nodeLoc{1, 0};

    bool result = astar::check_node_inside_map(nodeLoc, grid);

    EXPECT_TRUE(result);
}

TEST(AstarCheckNodeLocTest, GivenOutOfBoundsNodeLoc_WhenCheckedIfValidAndNotInClosedSet_ExpectFalse)
{
    my_math::Grid grid(2, std::vector<double>(2, 0));
    std::vector<std::shared_ptr<Node>> closedSet{};
    std::array<double, 2> nodeLoc{-1, 0};

    bool result = astar::check_node_inside_map(nodeLoc, grid);

    EXPECT_FALSE(result);
}

TEST(AstarCheckNodeObsTest, GivenMapWithObstacleNodeOnObstacle_WhenOnObsNodeChecked_ExpectTrue)
{
    my_math::Grid map = {{0, 0, 0, 0, 0},
                         {0, 0, 1, 0, 0},
                         {0, 0, 0, 0, 0},
                         {0, 0, 0, 0, 0},
                         {0, 0, 0, 0, 0}};
    Node nodeOnObstacle({1, 2}, 10, 20, nullptr);

    bool result = astar::check_node_is_on_obstacle(nodeOnObstacle.get_location(), map, 0);

    EXPECT_TRUE(result);
}


TEST(AstarCheckNodeObsTest, GivenMapWithObstacleNodeNearObstacle_WhenOnObsNodeChecked_ExpectFalse)
{
    my_math::Grid map = {{0, 0, 0, 0, 0},
                         {0, 0, 0, 0, 0},
                         {0, 0, 0, 1, 0},
                         {0, 0, 0, 0, 0},
                         {0, 0, 0, 0, 0}};

    Node node({0, 0}, 10, 20, nullptr);

    bool result = astar::check_node_is_on_obstacle(node.get_location(), map, 2);

    EXPECT_FALSE(result);
}


class AstarCheckUpdateOpenSetTestFixture: public ::testing::Test
{
protected:
    std::shared_ptr<Node> n1 = std::make_shared<Node>(std::array<double, 2>{100, 100}, 10.0, 20.0, nullptr);
    std::shared_ptr<Node> n2 = std::make_shared<Node>(std::array<double, 2>{150, 100}, 20.0, 20.0, nullptr);
    std::shared_ptr<Node> n3 = std::make_shared<Node>(std::array<double, 2>{100, 100}, 50.0, 22.0, nullptr);
    std::shared_ptr<Node> n4 = std::make_shared<Node>(std::array<double, 2>{100, 100}, 5.0, 20.0, nullptr);
    std::vector<std::shared_ptr<Node>> openSet{n1};
    double tol{1e-4};
};

TEST_F(AstarCheckUpdateOpenSetTestFixture, GivenNodeNotInOpenSet_WhenCheckOpenSetCalled_ExpectFalse)
{
    bool result = astar::check_node_in_set(openSet, n2->get_location());

    EXPECT_FALSE(result);
}

TEST_F(AstarCheckUpdateOpenSetTestFixture, GivenNodeInOpenSett_WhenCheckOpenSetCalled_ExpectTrue)
{
    bool result = astar::check_node_in_set(openSet, n3->get_location());

    EXPECT_TRUE(result);
}

TEST_F(AstarCheckUpdateOpenSetTestFixture, GivenNodeWithHigherCost_WhenUpdateOpenSetCalled_ExpectNodeNotUpdated)
{
    astar::update_openset_node(openSet, n3);

    EXPECT_NE(openSet[0]->get_costFromStart(), n3->get_costFromStart());
    EXPECT_NE(openSet[0]->get_costToGoal(), n3->get_costToGoal());
    EXPECT_NE(openSet[0]->get_totalCost(), n3->get_totalCost());
    EXPECT_EQ(openSet.size(), 1);
}

TEST_F(AstarCheckUpdateOpenSetTestFixture, GivenNodeWithLowerCost_WhenUpdateOpenSetCalled_ExpectNodeUpdated)
{
    astar::update_openset_node(openSet, n4);

    EXPECT_NEAR(openSet[0]->get_costFromStart(), n4->get_costFromStart(), tol);
    EXPECT_NEAR(openSet[0]->get_costToGoal(), n4->get_costToGoal(), tol);
    EXPECT_NEAR(openSet[0]->get_totalCost(), n4->get_totalCost(), tol);
    EXPECT_EQ(openSet[0]->get_parent(), n4->get_parent());
    EXPECT_EQ(openSet.size(), 1);
}

class AstarNewNeighborTest: public ::testing::Test
{
protected:
    std::array<double, 2> neighborLoc{1, 1};
    std::array<double, 2> currentLoc{0, 0};
    std::array<double,2> closerLoc{1,1};
    std::array<double, 2> goal{1, 2};
    std::shared_ptr<Node> current = std::make_shared<Node>(currentLoc, 10.0, my_math::calculate_xy_distance(currentLoc, goal), nullptr);
    std::vector<std::shared_ptr<Node>> openSet{current};
    double tol{1e-5};
};

TEST_F(AstarNewNeighborTest, GivenData_WhenNewNeighborCreated_ExpectCorrectData)
{
    std::shared_ptr<Node> newNeighbor = astar::create_new_neighbor(current, goal, neighborLoc);

    EXPECT_NEAR(newNeighbor->get_costToGoal(), 1.0, tol);
    EXPECT_NEAR(newNeighbor->get_costFromStart(), current->get_costFromStart()+1, tol);
    EXPECT_NEAR(newNeighbor->get_totalCost(), current->get_costFromStart()+2, tol);
    EXPECT_EQ(current, newNeighbor->get_parent());
}

TEST_F(AstarNewNeighborTest, GivenNodeNotInOpenSet_WhenInsertingNode_ExpectNodeInserted) {
    openSet.clear();
    astar::handle_new_neighbor(openSet, current);

    EXPECT_EQ(openSet.size(), 1);
    EXPECT_EQ(openSet[0]->get_location(), currentLoc);
    EXPECT_EQ(openSet[0]->get_costFromStart(), 10.0);
    EXPECT_EQ(openSet[0]->get_costToGoal(), my_math::calculate_xy_distance(currentLoc, goal));
}

TEST_F(AstarNewNeighborTest, GivenExistingNodeInOpenSet_WhenUpdatingExistingNode_ExpectExistingNodeUpdated) {
    std::shared_ptr<Node> newNeighbor = astar::create_new_neighbor(current, goal, neighborLoc);
    astar::handle_new_neighbor(openSet, newNeighbor);

    EXPECT_EQ(openSet.size(), 2);
    EXPECT_EQ(openSet[0]->get_location(), neighborLoc);
    EXPECT_EQ(openSet[0]->get_costFromStart(), 11.0);
    EXPECT_EQ(openSet[0]->get_costToGoal(), my_math::calculate_xy_distance(neighborLoc, goal));
}

TEST(AstarPathTest, GivenSolutionVector_WhenGetPathIsCalled_ExpectCorrectPathBack)
{
    Node n1{std::array<double, 2>{0,0}, 10.0, 20.0, nullptr};
    Node n2{std::array<double, 2>{1,1}, 10.0, 20.0, std::make_shared<Node>(n1)};
    Node n3{std::array<double, 2>{2,2}, 10.0, 20.0, std::make_shared<Node>(n2)};
    Node n4{std::array<double, 2>{3,3}, 10.0, 20.0, std::make_shared<Node>(n3)};
    std::vector<std::array<double, 2>> ground_truth{std::array<double, 2>{0,0}, std::array<double, 2>{1,1}, std::array<double, 2>{2,2}, std::array<double, 2>{3,3}};

    std::vector<std::array<double, 2>> path = astar::get_path_from_solution(std::make_shared<Node>(n4));

    EXPECT_TRUE(path == ground_truth);
}

TEST(ASTARFullAlgorithm, GivenOneByOneGridWithNoObstacles_WhenAStarRan_ExpectShortestPathToGoal)
{
    my_math::Grid grid(1, std::vector<double>(1, 0));
    std::array<double, 2> start{0, 0};
    std::array<double, 2> goal{0, 0};
    const int maxIterations{3};
    std::vector<std::array<double, 2>> ground_truth{std::array<double, 2>{0,0}};

    std::vector<std::array<double, 2>> solution_path = astar::astar(grid, start, goal, maxIterations);

    EXPECT_TRUE(solution_path == ground_truth);
}

TEST(ASTARFullAlgorithm, GivenSmallGridWithNoObstacles_WhenAStarRan_ExpectShortestPathToGoal)
{
    my_math::Grid grid(2, std::vector<double>(2, 0));
    std::array<double, 2> start{0, 0};
    std::array<double, 2> goal{1, 1};
    const int maxIterations{4};
    std::vector<std::array<double, 2>> ground_truth{std::array<double, 2>{0,0},
                                                    std::array<double, 2>{1,1}};

    std::vector<std::array<double, 2>> solution_path = astar::astar(grid, start, goal, maxIterations);

    EXPECT_TRUE(solution_path == ground_truth);
}

TEST(ASTARFullAlgorithm, GivenLargeGridWithNoObstacles_WhenAStarRan_ExpectShortestPathToGoal)
{
    my_math::Grid grid(500, std::vector<double>(600, 0));
    std::array<double, 2> start{0, 0};
    std::array<double, 2> goal{395, 560};
    const int maxIterations{10000};

    std::vector<std::array<double, 2>> solution_path = astar::astar(grid, start, goal, maxIterations);

    EXPECT_TRUE(solution_path.back() == goal);
}

TEST(ASTARFullAlgorithm, GivenLargeGridWithNoObstaclesAndTooFewIterations_WhenAStarRan_ExpectFinalWaypointCloserToGoal)
{
    my_math::Grid grid(500, std::vector<double>(600, 0));
    std::array<double, 2> start{0, 0};
    std::array<double, 2> goal{395, 560};
    const int maxIterations{300};

    std::vector<std::array<double, 2>> solution_path = astar::astar(grid, start, goal, maxIterations);

    EXPECT_TRUE(my_math::calculate_xy_distance(start, goal) >
                my_math::calculate_xy_distance(solution_path.back(), goal));
}

TEST(ASTARFullAlgorithm, GivenSmallGridWithObstacles_WhenAStarRan_ExpectShortestPathToGoal)
{
    my_math::Grid map = {
                         {0, 0, 0, 0, 0},
                         {0, 1, 1, 1, 0},
                         {0, 0, 1, 1, 0},
                         {0, 0, 1, 0, 1},
                         {0, 0, 1, 1, 0}};

    std::array<double, 2> start{0, 0};
    std::array<double, 2> goal{4, 4};
    const int maxIterations{10000};

    std::vector<std::array<double, 2>> solution_path = astar::astar(map, start, goal, maxIterations, 0);

    for (int i{0}; i < solution_path.size(); i++) {
        EXPECT_EQ(map[solution_path[i][0]][solution_path[i][1]], 0.0);
    }

    EXPECT_TRUE(my_math::calculate_xy_distance(solution_path.back(), goal) < 1e-2);
}

TEST(ASTARFullAlgorithm, GivenLargeGridWithObstacles_WhenAStarRan_ExpectShortestPathToGoal)
{
    Environment env{500, 600};

    std::array<double, 2> start{0, 0};
    std::array<double, 2> goal{395, 560};
    const int maxIterations{10000};

    env.set_goalLocation(goal);
    obs::generate_obstacles(env, {0, 0}, 1000, 4);

    std::vector<std::array<double, 2>> solution_path = astar::astar(env.get_map(), start, goal, maxIterations);

    for (int i{0}; i < solution_path.size(); i++) {
        EXPECT_EQ(env.get_map()[solution_path[i][0]][solution_path[i][1]], 0.0);
    }

    EXPECT_TRUE(my_math::calculate_xy_distance(solution_path.back(), goal) < 1e-2);
}


TEST(GetSparsePathTest, GivenEmptyPath_WhenSparsePathRequested_ExpectEmptyPath)
{
    std::vector<std::array<double, 2>> emptyPath{};
    int factor{2};

    std::vector<std::array<double, 2>> result = astar::get_sparse_path(emptyPath, factor);

    EXPECT_TRUE(result.empty());
}

TEST(GetSparsePathTest, GivenPathWithFactorTwo_WhenSparsePathRequested_ExpectEverySecondPoint)
{
    std::vector<std::array<double, 2>> path = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
    int factor{2};
    std::vector<std::array<double, 2>> expected = {{0, 0}, {2, 2}, {3, 3}};

    std::vector<std::array<double, 2>> result = astar::get_sparse_path(path, factor);

    EXPECT_EQ(result, expected);
}

TEST(GetSparsePathTest, GivenPathWithFactorThree_WhenSparsePathRequested_ExpectEveryThirdPoint)
{
    std::vector<std::array<double, 2>> path = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
    int factor{3};
    std::vector<std::array<double, 2>> expected = {{0, 0}, {3, 3}};

    std::vector<std::array<double, 2>> result = astar::get_sparse_path(path, factor);

    EXPECT_EQ(result, expected);
}
