#ifndef DRAWING_UTILS_H
#define DRAWING_UTILS_H

#include <QGraphicsPolygonItem>
#include <QPolygonF>
#include <QGraphicsItem>
#include <vector>
#include <array>

namespace draw
{
QGraphicsPolygonItem* draw_triangle(const std::array<double,3> & pose);
void update_triangle(QGraphicsPolygonItem* triangle, const std::array<double,3> & pose);
QGraphicsPolygonItem* draw_star(const std::array<double,2>& location);
void update_star(QGraphicsPolygonItem* star, const std::array<double,2>& location);
void plot_path(const std::vector<std::array<double,2>>& path, QGraphicsScene* scene);
void draw_obstacles_in_scene(QGraphicsScene*& scene, const std::vector<std::array<double,2>>& obstacles, const int& size);
};
#endif // DRAWING_UTILS_H
