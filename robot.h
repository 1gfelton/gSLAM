#ifndef ROBOT_H
#define ROBOT_H

#include "landmark.h"
#include "point2d.h"
#include "control.h"
#include "pose.h"
#include <vector>
#include <Eigen/Dense>

/*
Robot can either be 0 init, init with a location, or init with a trajectory
It needs a direction that its facing
`look_at` is the direction the robot is facing in DEGREES
`position` is of type `Eigen::Vector2d`
*/
struct Robot{
    Robot();
    Robot(const float _x, const float _y);
    Robot(const float _x, const float _y, const float _look_at);
    Robot(const std::vector<std::pair<Pose, Control>> t);

    /*
    Pose: x, y, \theta
    */
    float x; float y; float look_at;
    Eigen::Vector2d position;
    std::vector<std::pair<Pose, Control>> trajectory;
    void print() const;

    // move in a direction with some magnitude
    void move_in_direction(Control u, Pose p);
    float distance_to(Landmark landmark);
    void generate_lerp_trajectory(Point2d start, Point2d end, int n_steps);
};

#endif