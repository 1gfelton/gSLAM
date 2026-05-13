#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>

struct Pose {
    Pose();
    Pose(Eigen::Vector2d _position, double _orientation);
    Eigen::Vector2d position;
    double orientation;
    void print() const;
};

#endif