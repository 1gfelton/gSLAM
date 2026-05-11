#include <iostream>
#include "robot.h"
#include "point2d.h"
using std::cout;

// atan2 convenience func because Eigen doesn't have one
// float my_atan2(float a, float b) { return std::atan2(a, b); }

// Init Methods
Robot::Robot() : x(0.0), y(0.0) {}
Robot::Robot(const float _x, const float _y) : x(_x), y(_y), position(_x, _y) {}
Robot::Robot(const float _x, const float _y, const float _look_at) : x(_x), y(_y), look_at(_look_at), position(_x, _y) {}
// set position to the most recent point in the trajectory
Robot::Robot(const std::vector<Point2d> t) : trajectory(t), position(t[t.size() - 1].coords), x(t[t.size() - 1].x), y(t[t.size() - 1].y) {}

void Robot::move_in_direction(Eigen::Vector2f dir, float mag) {
    Eigen::Vector2f delta = dir - this->position;
}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << x << ", " << y << ")\n";
}