#ifndef GUI_UTILS_H
#define GUI_UTILS_H

#include <QPolygonF>

namespace gui
{
std::vector<std::array<double, 2>> convert_to_vector(const QPolygonF& polygon);
}

#endif // GUI_UTILS_H
