#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <array>

#include "node.hpp"
#include "math_utils.hpp"

namespace astar
{
const std::array<std::array<double, 2>, 8> MOVE_OPTIONS{{{-1,1}, {0,1}, {1,1},
                                                         {-1,0},        {1,0},
                                                         {-1,-1},{0,-1},{1,-1}}};

void insert_into_openset(std::vector<std::shared_ptr<Node>>& openSet, std::shared_ptr<Node>& n);
bool check_node_inside_map(const std::array<double, 2>& nodeLoc, const my_math::Grid& currMap);
bool check_node_is_on_obstacle(const std::array<double, 2>& nodeLoc, const my_math::Grid& currMap, const int& tol);
bool check_node_in_set(const std::vector<std::shared_ptr<Node>>& set, const std::array<double, 2>&  nodeLoc);
void update_openset_node(std::vector<std::shared_ptr<Node>>& openSet, const std::shared_ptr<Node>& neighborNode);
std::shared_ptr<Node> create_new_neighbor(std::shared_ptr<Node>& current, const std::array<double,2>& goal, std::array<double,2>& neighborLoc);
void handle_new_neighbor(std::vector<std::shared_ptr<Node>>& openSet, std::shared_ptr<Node>& newNeighbor);
std::vector<std::array<double, 2>> get_path_from_solution(std::shared_ptr<Node> n);
std::vector<std::array<double, 2>> astar(const my_math::Grid& grid, const std::array<double, 2>& start, const std::array<double, 2>& goal, const int& maxIterations, const int& tol=1);
std::vector<std::array<double, 2>> get_sparse_path(const std::vector<std::array<double,2>>& path, const int& factor);
};

#endif // ASTAR_H
