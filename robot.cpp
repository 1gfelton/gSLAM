#include <iostream>
#include <math.h>
#include "robot.h"
#include "point2d.h"
#include "utils.h"
#include "control.h"
#include "pose.h"
using std::cout;
using std::vector;
using std::pair;

// Init Methods
Robot::Robot() : x(0.0), y(0.0), look_at(0.0), trajectory({make_traj_position(Eigen::Vector2d(), 0.0, 0.0, 0.0)}) {}
// Init trajectory as haivng the init position as the first position
Robot::Robot(const float _x, const float _y) : x(_x), y(_y), position(_x, _y), trajectory({make_traj_position(Eigen::Vector2d(), 0.0, 0.0, 0.0)}) {}
Robot::Robot(const float _x, const float _y, const float _look_at) : x(_x), y(_y), look_at(_look_at), position(_x, _y), trajectory({make_traj_position(Eigen::Vector2d(), 0.0, 0.0, 0.0)}) {}
// set position to the most recent point in the trajectory
Robot::Robot(const vector<pair<Pose, Control>> t) : trajectory(t), position(t[t.size() - 1].first.position), x(t[t.size() - 1].first.position.x()), y(t[t.size() - 1].first.position.y()), look_at(0.0) {}

/*
TODO add gaussian noise to the movement
$\hat{v} = v + \text{sample}(\alpha_1v^2 + \alpha_2\omega^2)$
*/ 

/* feel like trajectory should instead be vector<pair<Pose, Control>> where 
Pose is (x, y, theta), Control is (v, w) or translational velocity and angular velocity*/
void Robot::move_in_direction(float dist) {
    /*
    move to `look_at` some distance `dist`
    */
    Eigen::Vector2d delta = {cosd(this->look_at), sind(this->look_at)};
    this->position += (dist * delta);

    // now add this new direction to the trajectory
    this->trajectory.push_back(make_traj_position(this->position, this->look_at, dist, 0.0));
}

float Robot::distance_to(Landmark landmark) {
    return abs((landmark.position - this->position).norm());
}

void Robot::generate_lerp_trajectory(Point2d start, Point2d end, int n_steps) {
    /*
    uses lerp to generate a linear trajectory from `start` to `end`
    */
    Eigen::Vector2d start_pos(start.x, start.y);
    double total_dist = start.distance_to(end);
    double step_size = total_dist / n_steps;
    this->trajectory = {make_traj_position(start_pos, this->look_at, step_size, 0.0)};
    Eigen::Vector2d cur_pos = this->position;
    for (int i = 0; i < n_steps; i++) {
        move_in_direction(step_size);
    }
}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << position.x() << ", " << position.y() << ") look_at: " << look_at << std::endl;
}