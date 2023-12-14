#include "astar.hpp"

#include <algorithm>

namespace astar
{

void insert_into_openset(std::vector<std::shared_ptr<Node>>& openSet, std::shared_ptr<Node>& n)
{
    for (int i{0}; i<openSet.size(); i++)
    {
        if (openSet[i]->get_totalCost() > n->get_totalCost())
        {
            openSet.insert(openSet.begin()+i, n);
            return;
        }
    }
    openSet.push_back(n);
}

bool check_node_inside_map(const std::array<double, 2>& nodeLoc, const my_math::Grid& currMap)
{
    if (nodeLoc[0] < 0 || nodeLoc[0] >= currMap.size() ||
        nodeLoc[1] < 0 || nodeLoc[1] >= currMap[0].size())
    {
        return false;
    }
    return true;
}

bool check_node_is_on_obstacle(const std::array<double, 2>& nodeLoc, const my_math::Grid& currMap, const int& tol)
{
    if (currMap[nodeLoc[0]][nodeLoc[1]] == 1.0)
    {
        return true;
    }

    for (int i{-tol}; i <= tol; i++)
    {
        for (int j{-tol}; j <= tol; j++)
        {
            int x{static_cast<int>(nodeLoc[0]) + i};
            int y{static_cast<int>(nodeLoc[1]) + j};
            if (x >= 0 && x < currMap.size() && y >= 0 && y < currMap[0].size())
            {
                if (currMap[x][y] == 1)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool check_node_in_set(const std::vector<std::shared_ptr<Node>>& set,  const std::array<double,2>& nodeLoc)
{
    auto it = std::find_if(set.begin(), set.end(), [&](const auto& node) {
        return my_math::are_vectors_equal(node->get_location(), nodeLoc);
    });
    return it != set.end();
}

void update_openset_node(std::vector<std::shared_ptr<Node>>& openSet, const std::shared_ptr<Node>& neighborNode)
{
    auto it = std::find_if(openSet.begin(), openSet.end(), [&](const auto& node) {
        return my_math::are_vectors_equal(node->get_location(), neighborNode->get_location()) &&
               node->get_totalCost() > neighborNode->get_totalCost();
    });

    if (it != openSet.end())
    {
        (*it)->set_costFromStart(neighborNode->get_costFromStart());
        (*it)->set_costToGoal(neighborNode->get_costToGoal());
        (*it)->set_parent(neighborNode->get_parent());
    }
}

std::shared_ptr<Node> create_new_neighbor(std::shared_ptr<Node>& current, const std::array<double,2>& goal, std::array<double,2>& neighborLoc)
{
    double newCostFromStart{current->get_costFromStart() + 1};
    double newCostToGoal = my_math::calculate_xy_distance(neighborLoc, goal);
    std::shared_ptr<Node> neighborNode = std::make_shared<Node>(neighborLoc, newCostFromStart, newCostToGoal, current);
    return neighborNode;
}

void handle_new_neighbor(std::vector<std::shared_ptr<Node>>& openSet, std::shared_ptr<Node>& newNeighbor)
{
    if (check_node_in_set(openSet, newNeighbor->get_location()))
    {
        update_openset_node(openSet, newNeighbor);
    }
    else
    {
        insert_into_openset(openSet, newNeighbor);
    }
}

std::vector<std::array<double, 2>> get_path_from_solution(std::shared_ptr<Node> n)
{
    std::vector<std::array<double, 2>> path{};
    while (nullptr != n)
    {
        path.insert(path.begin(), n->get_location());
        n = n->get_parent();
    }
    return path;
}

std::vector<std::array<double, 2>> astar(const my_math::Grid& currMap, const std::array<double, 2>& start, const std::array<double, 2>& goal, const int& maxIterations, const int& tol)
{
    std::shared_ptr<Node> startNode = std::make_shared<Node>(start, 0, my_math::calculate_xy_distance(start, goal), nullptr);
    std::vector<std::shared_ptr<Node>> openSet{startNode};
    std::vector<std::shared_ptr<Node>> closedSet{};

    openSet.reserve(static_cast<int>(goal[0]*goal[1]));
    closedSet.reserve(static_cast<int>(goal[0]*goal[1]));

    int i{0};
    while (!openSet.empty() && i < maxIterations)
    {
        std::shared_ptr<Node> current{openSet.front()};
        if (my_math::are_vectors_equal(current->get_location(), goal)) {
            break;
        }
        closedSet.push_back(current);
        openSet.erase(openSet.begin());

        for (const std::array<double, 2>& deltaMove : MOVE_OPTIONS)
        {
            std::array<double, 2> neighborLoc = current->get_location();
            neighborLoc[0] += deltaMove[0];
            neighborLoc[1] += deltaMove[1];
            if (check_node_inside_map(neighborLoc, currMap) &&
                !check_node_is_on_obstacle(neighborLoc, currMap, tol) &&
                !check_node_in_set(closedSet, neighborLoc))
            {
                std::shared_ptr<Node> newNeighbor = create_new_neighbor(current, goal, neighborLoc);
                handle_new_neighbor(openSet, newNeighbor);
            }
        }
        i++;
    }
    std::vector<std::array<double, 2>> path{};
    if (!openSet.empty())
    {
        path = get_path_from_solution(openSet.front());
    }
    return path;
}

std::vector<std::array<double,2>> get_sparse_path(const std::vector<std::array<double,2>>& path, const int& factor)
{
    if (path.empty())
    {
        return path;
    }

    std::vector<std::array<double,2>> sparse_path{};
    for (int i{0}; i < path.size(); i+=factor)
    {
        sparse_path.push_back(path[i]);
    }

    if (sparse_path.back() != path.back())
    {
        sparse_path.push_back(path.back());
    }
    return sparse_path;
}


};
