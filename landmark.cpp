#include "landmark.h"
#include <random>
#include <iostream>
#include <Eigen/Dense>

Landmark::Landmark() : x(0.0), y(0.0) {}
Landmark::Landmark(float _x, float _y) : x(_x), y(_y), coords({_x, _y}) {}
Landmark::Landmark(Eigen::Vector2f _coords) : coords(_coords), x(_coords[0]), y(_coords[1]) {}

// Randomize a pre-initialized Landmark
void Landmark::randomize(const float &lower_bound, const float &upper_bound) {
    static std::random_device r;
    static std::mt19937 generate(r());
    std::uniform_real_distribution<float> P(lower_bound, upper_bound);
    x = P(generate);
    y = P(generate);
    coords = Eigen::Vector2f({x, y});
}

// Print a Landmark to console
void Landmark::print() const {
    std::cout << "Landmark: (" << x << ", " << y << ")\n";
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
    float eps = 1e-4;
    float dx = std::abs(a.x - b.x);
    float dy = std::abs(a.y - b.y);
    return std::tie(dx, dy) < std::tie(eps, eps);
}