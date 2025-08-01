#include "node.hpp"

Node::Node(const std::array<double, 2> &loc, double g_val, double h_val, std::shared_ptr<Node> parent_node)
    : location(loc), costFromStart(g_val), costToGoal(h_val), parent(parent_node) {}

std::array<double, 2> Node::get_location() const
{
    return location;
}

double Node::get_costFromStart() const
{
    return costFromStart;
}

double Node::get_costToGoal() const
{
    return costToGoal;
}

double Node::get_totalCost() const
{
    return costFromStart + costToGoal;
}

std::shared_ptr<Node> Node::get_parent() const
{
    return parent;
}

void Node::set_location(const std::array<double, 2> &newLocation)
{
    location = newLocation;
}

void Node::set_costFromStart(const double& newCostFromStart)
{
    costFromStart = newCostFromStart;
}

void Node::set_costToGoal(const double& newCostToGoal)
{
    costToGoal = newCostToGoal;
}

void Node::set_parent(const std::shared_ptr<Node>& newParent)
{
    parent = newParent;
}

bool Node::operator==(const Node& other) const
{
    return (
        location == other.location &&
        costFromStart == other.costFromStart &&
        costToGoal == other.costToGoal &&
        parent == other.parent
        );
}

bool Node::operator!=(const Node& other) const
{
    return !(*this == other);
}
