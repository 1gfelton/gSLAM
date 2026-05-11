#ifndef ROBOT_H
#define ROBOT_H

#include "landmark.h"
#include "point2d.h"
#include <vector>
#include <Eigen/Dense>

/*
Robot can either be 0 init, init with a location, or init with a trajectory
It needs a direction that its facing
*/
struct Robot{
    Robot();
    Robot(const float _x, const float _y);
    Robot(const float _x, const float _y, const float _look_at);
    Robot(const std::vector<Point2d> t);
    float x; float y; float look_at;
    Eigen::Vector2f position;
    std::vector<Point2d> trajectory;
    void print() const;

    // move in a direction with some magnitude
    void move_in_direction(Eigen::Vector2f dir, float mag);
};

#endif