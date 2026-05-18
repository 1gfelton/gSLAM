#include "landmark.h"
#include <random>
#include <iostream>
#include <Eigen/Dense>

Landmark::Landmark() : x(0.0), y(0.0) {}
Landmark::Landmark(double _x, double _y) : x(_x), y(_y), position({_x, _y}) {}
Landmark::Landmark(Eigen::Vector2d _position) : position(_position), x(_position.x()), y(_position.y()) {}

// Randomize a pre-initialized Landmark
void Landmark::randomize(const double &lower_bound, const double &upper_bound) {
    static std::random_device r;
    static std::mt19937 generate(r());
    std::uniform_real_distribution<double> P(lower_bound, upper_bound);
    x = P(generate);
    y = P(generate);
    position = Eigen::Vector2d({x, y});
}

// Print a Landmark to console
void Landmark::print() const {
    std::cout << "Landmark: (" << position.x() << ", " << position.y() << ")\n";
}

bool operator<(const Landmark& a, const Landmark& b) {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

bool operator<=(const Landmark& a, const Landmark& b) {
    return std::tie(a.x, a.y) <= std::tie(b.x, b.y);
}

bool operator>(const Landmark& a, const Landmark& b) {
    return std::tie(a.x, a.y) > std::tie(b.x, b.y);
}

bool operator>=(const Landmark& a, const Landmark& b) {
    return std::tie(a.x, a.y) >= std::tie(b.x, b.y);
}

bool operator==(const Landmark& a, const Landmark& b) { 
    double eps = 1e-4;
    double dx = std::abs(a.x - b.x);
    double dy = std::abs(a.y - b.y);
    return std::tie(dx, dy) < std::tie(eps, eps);
}