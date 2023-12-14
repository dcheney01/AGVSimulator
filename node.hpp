#ifndef NODE_H
#define NODE_H

#include <array>
#include <memory>

class Node
{
public:
    Node(const std::array<double, 2>& loc, double g_val, double h_val, std::shared_ptr<Node> parent_node);

    std::array<double, 2> get_location() const;
    double get_costFromStart() const;
    double get_costToGoal() const;
    double get_totalCost() const;
    std::shared_ptr<Node> get_parent() const;

    void set_location(const std::array<double, 2> &newLocation);
    void set_costFromStart(const double& newCostFromStart);
    void set_costToGoal(const double& newCostToGoal);
    void set_parent(const std::shared_ptr<Node>& newParent);

    bool operator==(const Node& other) const;
    bool operator!=(const Node& other) const;

protected:
    std::array<double, 2> location{0,0};
    double costFromStart{0.0};
    double costToGoal{0.0};
    std::shared_ptr<Node> parent{nullptr};
};
#endif // NODE_H
