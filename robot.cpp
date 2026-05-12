#include <iostream>
#include <math.h>
#include "robot.h"
#include "point2d.h"
#include "utils.h"
using std::cout;
using std::vector;

// Init Methods
Robot::Robot() : x(0.0), y(0.0), look_at(0.0), trajectory({Point2d(0.0, 0.0)}) {}
// Init trajectory as haivng the init position as the first position
Robot::Robot(const float _x, const float _y) : x(_x), y(_y), position(_x, _y), trajectory({Point2d(_x, _y)}) {}
Robot::Robot(const float _x, const float _y, const float _look_at) : x(_x), y(_y), look_at(_look_at), position(_x, _y), trajectory({Point2d(_x, _y)}) {}
// set position to the most recent point in the trajectory
Robot::Robot(const vector<Point2d> t) : trajectory(t), position(t[t.size() - 1].position), x(t[t.size() - 1].x), y(t[t.size() - 1].y), look_at(0.0) {}

void Robot::move_in_direction(float dist) {
    /*
    move to `look_at` some distance `dist`
    */
    // double in_radians = to_radians((double)this->look_at);
    Eigen::Vector2f delta = {cosd(this->look_at), sind(this->look_at)};
    this->position += (dist * delta);

    // now add this new direction to the trajectory
    this->trajectory.push_back(Point2d(this->position.x(), this->position.y()));
}

float Robot::distance_to(Landmark landmark) {
    return abs((landmark.position - this->position).norm());
}

void Robot::generate_lerp_trajectory(Point2d start, Point2d end, int n_steps) {
    /*
    uses lerp to generate a linear trajectory from `start` to `end`
    */
    this->trajectory = {start};
    float total_dist = start.distance_to(end);
    double step_size = total_dist / n_steps;
    Eigen::Vector2f cur_pos = this->position;
    for (int i = 0; i < n_steps; i++) {
        move_in_direction(step_size);
    }
}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << position.x() << ", " << position.y() << ") look_at: " << look_at << std::endl;
}