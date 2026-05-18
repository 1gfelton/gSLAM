#include "point2d.h"
#include "utils.h"
#include <random>
#include <iostream>
#include <Eigen/Dense>

Point2d::Point2d() : x(0.0), y(0.0), position(Eigen::Vector2d(0.0, 0.0)) {}                     // zero init
Point2d::Point2d(double _x, double _y) : x(_x), y(_y), position(Eigen::Vector2d({_x, _y})){}      // init with x & y values
Point2d::Point2d(Eigen::Vector2d _v) : position(_v), x(_v.x()), y(_v.y()) {}                    // init with Eigen vector

void Point2d::randomize(const double &lower_bound, const double &upper_bound) {
    double x = sample_triangular_dist(0, 1);
    double y = sample_triangular_dist(0, 1);
    position = Eigen::Vector2d({x, y});
}

void Point2d::print() const {
    std::cout << "Point2d: (" << position.x() << ", " << position.y() << ")\n";
}

double Point2d::distance_to(Point2d other) {
    return abs((this->position - other.position).norm());
}

bool operator<(const Point2d& a, const Point2d& b) {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

bool operator<=(const Point2d& a, const Point2d& b) {
    return std::tie(a.x, a.y) <= std::tie(b.x, b.y);
}

bool operator>(const Point2d& a, const Point2d& b) {
    return std::tie(a.x, a.y) > std::tie(b.x, b.y);
}

bool operator>=(const Point2d& a, const Point2d& b) {
    return std::tie(a.x, a.y) >= std::tie(b.x, b.y);
}

bool operator==(const Point2d& a, const Point2d& b) { 
    double eps = 1e-4;
    double dx = std::abs(a.x - b.x);
    double dy = std::abs(a.y - b.y);
    return std::tie(dx, dy) < std::tie(eps, eps);
}