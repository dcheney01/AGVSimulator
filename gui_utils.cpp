#include "gui_utils.h"

#include <array>
namespace gui
{
std::vector<std::array<double, 2>> convert_to_vector(const QPolygonF& polygon) {
    std::vector<std::array<double, 2>> result;

    for (const QPointF& point : polygon) {
        result.push_back({point.x(), point.y()});
    }
    return result;
}
}
