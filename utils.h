#pragma once
#include <Eigen/Dense>
#include "control.h"
#include "pose.h"
double to_radians(float deg);
double sind(double x);
double cosd(double x);
bool isclose(double a, double b, double rtol = 1e-5, double atol = 1e-8);
std::pair<Pose, Control> make_traj_position(Eigen::Vector2d pos, double theta, double v, double w);