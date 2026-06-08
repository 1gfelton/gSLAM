#pragma once
#include <Eigen/Dense>
#include <random>
#include <fstream>
#include <string>
#include "control.h"
#include "pose.h"
double to_radians(double deg);
bool isclose(double a, double b, double rtol = 1e-8, double atol = 1e-10);
double triangle_distribution(double a, double mu, double sigma);
std::pair<Pose, Control> make_traj_position(Eigen::Vector2d pos, double theta, double v, double w);
std::piecewise_linear_distribution<double> triangular_distribution(double mu, double sigma2);
double sample_triangular_dist(double mu, double sigma2);
double triangular_prob(double mu, double sigma2);
Pose make_pose(Eigen::Vector3d v);
Eigen::VectorXd to_cartesian(Eigen::VectorXd z);

template <typename Derived>
std::string to_str(const Eigen::DenseBase<Derived> &m) {
    std::ostringstream oss; oss << m; return oss.str();
}

template <typename Derived>
std::string shape(const Eigen::DenseBase<Derived> &m) {
    std::ostringstream oss; oss << "[" << m.rows() << ", " << m.cols() << "]\n"; return oss.str();
}
template <typename Derived1, typename Derived2>
Eigen::VectorXd to_cartesian(const Eigen::DenseBase<Derived1> &z, const Eigen::DenseBase<Derived2> &pos) {
    Eigen::VectorXd ans = Eigen::VectorXd::Zero(z.size());
    for (int i = 0; i < z.size(); i+= 3) {
        double r = z(i);
        double phi = z(i + 1);
        double s = z(i + 2);

        double x = pos(0) + r * cos(phi);
        double y = pos(1) + r * sin(phi);
        ans(Eigen::seq(i, i + 2)) = Eigen::Vector3d{x, y, s};
    }
    return ans; 
}