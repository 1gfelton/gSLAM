#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>

/*

$\begin{bmatrix}x_t\\y_t\\\theta\end{bmatrix}$

*/
struct Pose {
    Pose();
    Pose(Eigen::Vector2d _position, double _orientation);
    Eigen::Vector2d position;
    double orientation;
    void print() const;
};

#endif