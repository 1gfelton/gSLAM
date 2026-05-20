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
    Robot(const double _x, const double _y);
    Robot(const double _x, const double _y, const double _look_at);
    Robot(const std::vector<std::pair<Pose, Control>> t);

    /*
    look_at = theta
    position = (x, y)
    */
    double look_at;
    Eigen::Vector2d position;
    std::vector<std::pair<Pose, Control>> trajectory;
    Eigen::VectorXd state_vec;
    void print() const;

    // move in a direction with some magnitude
    Pose sample_xt(Control u, Pose p);
    double get_motion_probability(Pose x, Control u, Pose prev);
    void EKF_SLAM(Eigen::Vector<double, 33> mu_p, Eigen::Matrix<double, 33, 33> cov_p, Eigen::Vector<double, 3> u_t, Eigen::Matrix<double, 2, 10> z_t, Eigen::Vector<int, 10 + 1> c_t);
    void Robot::update_state_vec(Eigen::MatrixXd features);
    Eigen::MatrixXd sense_env(Eigen::MatrixXd landmarks);
    double distance_to(Landmark landmark);
    void look_to(Point2d point);
    void move_to_new_pose(Pose p, Control u);
    void generate_lerp_trajectory(Point2d start, Point2d end, int n_steps);
    void write_traj_to_csv();
};

#endif