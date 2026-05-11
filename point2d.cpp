#include "point2d.h"
#include <random>
#include <iostream>
#include <Eigen/Dense>

Point2d::Point2d() : x(0.0), y(0.0) {}                      // zero init
Point2d::Point2d(float _x, float _y) : x(_x), y(_y) {}      // init with x & y values
Point2d::Point2d(Eigen::Vector2f _v) : position(_v) {}        // init with Eigen vector

void Point2d::randomize(const float &lower_bound, const float &upper_bound) {
    static std::random_device r;
    static std::mt19937 generate(r());
    std::uniform_real_distribution<float> P(lower_bound, upper_bound);
    x = P(generate);
    y = P(generate);
    position = Eigen::Vector2f({x, y});
}

void Point2d::print() const {
    std::cout << "Point2d: (" << position.x() << ", " << position.y() << ")\n";
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
    float eps = 1e-4;
    float dx = std::abs(a.x - b.x);
    float dy = std::abs(a.y - b.y);
    return std::tie(dx, dy) < std::tie(eps, eps);
}