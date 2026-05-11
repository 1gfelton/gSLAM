#include <iostream>
#include <math.h>
#include "robot.h"
#include "point2d.h"
#include "utils.h"
using std::cout;

// Init Methods
Robot::Robot() : x(0.0), y(0.0) {}
// Init trajectory as haivng the init position as the first position
Robot::Robot(const float _x, const float _y) : x(_x), y(_y), position(_x, _y), trajectory({Point2d(_x, _y)}) {}
Robot::Robot(const float _x, const float _y, const float _look_at) : x(_x), y(_y), look_at(_look_at), position(_x, _y), trajectory({Point2d(_x, _y)}) {}
// set position to the most recent point in the trajectory
Robot::Robot(const std::vector<Point2d> t) : trajectory(t), position(t[t.size() - 1].position), x(t[t.size() - 1].x), y(t[t.size() - 1].y) {}

void Robot::move_in_direction(float dist) {
    /*
    move to `look_at` some distance `dist`
    */
    double in_radians = to_radians((double)this->look_at);
    Eigen::Vector2f delta = {dist * cosd(in_radians), dist * sind(in_radians)};
    this->position += delta;

    // now add this new direction to the trajectory
    this->trajectory.push_back(Point2d(this->position.x(), this->position.y()));
}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << x << ", " << y << ")\n";
}