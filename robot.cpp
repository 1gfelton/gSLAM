#include <iostream>
#include "robot.h"
using std::cout;

// Init Methods
Robot::Robot() : x(0.0), y(0.0) {}
Robot::Robot(const float _x, const float _y) : x(_x), y(_y) {}
Robot::Robot(const float _x, const float _y, const float _look_at) : x(_x), y(_y), look_at(_look_at) {}
Robot::Robot(const std::vector<Landmark> t) : trajectory(t) {}

void Robot::move_in_direction() {

}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << x << ", " << y << ")\n";
}