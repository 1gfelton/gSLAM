#ifndef ROBOT_H
#define ROBOT_H

#include "landmark.h"
#include "point2d.h"
#include "control.h"
#include "pose.h"
#include <vector>
#include <map>
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
    std::map<int, std::vector<int>> observations{}; // j -> x_t (features -> pose at which feature was observed)
    Eigen::VectorXd state_vec;
    Eigen::MatrixXd covariance;
    void print() const;

    void EKF_SLAM(Eigen::VectorXd mu_p, Eigen::MatrixXd cov_p, Eigen::Vector2d u_t, Eigen::VectorXd z_t, Eigen::VectorXi c_t);
    void update_state_vec(Eigen::MatrixXd features);
    void look_to(Point2d point);
    void move_to_new_pose(Pose p, Control u);
    void generate_lerp_trajectory(Point2d start, Point2d end, int n_steps);
    void write_traj_to_csv();

    Eigen::Vector3d get_next_pose(Eigen::Vector3d mu_p, Eigen::Vector2d u);
    Eigen::Vector3d sample_xt(Control u, Pose p);
    Eigen::VectorXd sense_env(Eigen::MatrixXd landmarks, int t);

    Eigen::VectorXd Graph_SLAM(Eigen::VectorXd u, std::vector<Eigen::VectorXd> z_t, Eigen::VectorXi c_t);
    Eigen::VectorXd Graph_SLAM_init(Eigen::VectorXd u_t);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Graph_SLAM_linearize(Eigen::VectorXd u_t, std::vector<Eigen::VectorXd> z_t, Eigen::VectorXi c_t, Eigen::VectorXd mu);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Graph_SLAM_reduce(Eigen::MatrixXd omega, Eigen::MatrixXd xi);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Graph_SLAM_solve(Eigen::MatrixXd omega_, Eigen::MatrixXd xi_, Eigen::MatrixXd omega, Eigen::MatrixXd xi);

    double get_motion_probability(Pose x, Control u, Pose prev);
    double distance_to(Landmark landmark);
};

#endif