#include "drawing_utils.hpp"

#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QBrush>
#include <iostream>

#include "math_utils.hpp"
#include "gui_utils.h"

namespace draw
{
QGraphicsPolygonItem* draw_triangle(const std::array<double,3> & pose)
{
    QPolygonF triangle;
    const double hypotenuse_length{std::sqrt(60)};
    const double other_distance{std::sqrt(20)};
    triangle << QPointF(0, 0) << QPointF(hypotenuse_length, 0) << QPointF(other_distance,other_distance);
    QGraphicsPolygonItem* triangleItem = new QGraphicsPolygonItem(triangle);

    QBrush brush = Qt::SolidPattern;
    brush.setColor(Qt::blue);
    triangleItem->setBrush(brush);

    update_triangle(triangleItem, pose);
    triangleItem->setTransformOriginPoint(triangleItem->boundingRect().center());
    return triangleItem;
}

void update_triangle(QGraphicsPolygonItem* triangle, const std::array<double,3>& pose)
{
    std::array<double,2> centroid = my_math::find_polygon_centroid(gui::convert_to_vector(triangle->polygon()));
    triangle->setPos(pose[0] - centroid[0], pose[1] - centroid[1]);
    triangle->setRotation(my_math::rad2deg(pose[2]) - 90);
}

QGraphicsPolygonItem* draw_star(const std::array<double,2>& location)
{
    const double size{5.0};
    QPolygonF star;
    const double angleIncrement{360.0 / 5};

    for (int i{0}; i < 5; i++)
    {
        star << QPointF(size * qCos(qDegreesToRadians(i * angleIncrement)),
                        size * qSin(qDegreesToRadians(i * angleIncrement)));
        star << QPointF((size / 3.0) * qCos(qDegreesToRadians((i + 0.5) * angleIncrement)),
                        (size / 3.0) * qSin(qDegreesToRadians((i + 0.5) * angleIncrement)));
    }

    std::array<double,2> centroid = my_math::find_polygon_centroid(gui::convert_to_vector(star));
    QGraphicsPolygonItem* starItem = new QGraphicsPolygonItem(star);
    starItem->setPos(location[0] - centroid[0], location[1] - centroid[1]);
    QBrush brush = Qt::SolidPattern;
    brush.setColor(QColor(255, 0, 0, 255));
    starItem->setBrush(brush);
    return starItem;
}

void update_star(QGraphicsPolygonItem* star, const std::array<double,2>& location)
{
    std::array<double,2> centroid = my_math::find_polygon_centroid(gui::convert_to_vector(star->polygon()));
    star->setPos(location[0] - centroid[0], location[1] - centroid[1]);
}

void plot_path(const std::vector<std::array<double, 2>>& path, QGraphicsScene* scene)
{
    QPen pen(Qt::red);
    for (const auto &point : path)
    {
        QGraphicsEllipseItem *pointItem = new QGraphicsEllipseItem(point[0], point[1], 1.0, 1.0);
        pointItem->setPen(pen);
        scene->addItem(pointItem);
    }
}

void draw_obstacles_in_scene(QGraphicsScene*& scene, const std::vector<std::array<double,2>>& obstacles, const int& size)
{
    for (auto obs : obstacles)
    {
        QGraphicsEllipseItem* obstacleDot = new QGraphicsEllipseItem(obs[0], obs[1], size, size);
        obstacleDot->setBrush(Qt::black);
        scene->addItem(obstacleDot);
    }
}
};
