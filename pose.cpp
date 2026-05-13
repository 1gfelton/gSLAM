#include "pose.h"
#include <Eigen/Dense>
#include <iostream>
using std::cout;

Pose::Pose() {};
Pose::Pose(Eigen::Vector2d _position, double _orientation) : position(_position), orientation(_orientation) {};
void Pose::print() const {
    cout << "Pose: (" << this->position.x() << ", " << this->position.y() << ", " << this->orientation << ")\n";
};