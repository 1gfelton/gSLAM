#pragma once
#include <Eigen/Dense>
#include <random>
#include <fstream>
#include <string>
#include "control.h"
#include "pose.h"
double to_radians(double deg);
bool isclose(double a, double b, double rtol = 1e-5, double atol = 1e-8);
double triangle_distribution(double a, double mu, double sigma);
std::pair<Pose, Control> make_traj_position(Eigen::Vector2d pos, double theta, double v, double w);
std::piecewise_linear_distribution<double> triangular_distribution(double mu, double sigma2);
double sample_triangular_dist(double mu, double sigma2);
double triangular_prob(double mu, double sigma2);
Pose make_pose(Eigen::Vector3d v);

template <typename Derived>
std::string to_str(const Eigen::DenseBase<Derived> &m) {
    std::ostringstream oss; oss << m; return oss.str();
}