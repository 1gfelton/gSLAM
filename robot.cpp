#include <iostream>
#include <fstream>
#include <math.h>
#include "robot.h"
#include "point2d.h"
#include "utils.h"
#include "control.h"
#include "pose.h"
#include "config.h"

using std::cout;
using std::vector;
using std::pair;
using namespace CONFIG;

// Init Methods
Robot::Robot() : x(0.0), y(0.0), look_at(0.0), trajectory({make_traj_position(Eigen::Vector2d(), 0.0, 0.0, 0.0)}) {}
// Init trajectory as haivng the init position as the first position
Robot::Robot(const float _x, const float _y) : x(_x), y(_y), position(_x, _y), trajectory({make_traj_position(Eigen::Vector2d(), 0.0, 0.0, 0.0)}) {}
Robot::Robot(const float _x, const float _y, const float _look_at) : x(_x), y(_y), look_at(_look_at), position(_x, _y), trajectory({make_traj_position(Eigen::Vector2d(), 0.0, 0.0, 0.0)}) {}
// set position to the most recent point in the trajectory
Robot::Robot(const vector<pair<Pose, Control>> t) : trajectory(t), position(t[t.size() - 1].first.position), x(t[t.size() - 1].first.position.x()), y(t[t.size() - 1].first.position.y()), look_at(0.0) {}

/*
TODO add gaussian noise to the movement
$\hat{v} = v + \text{sample}(\alpha_1v^2 + \alpha_2\omega^2)$
*/ 

/* feel like trajectory should instead be vector<pair<Pose, Control>> where 
Pose is (x, y, theta), Control is (v, w) or translational velocity and angular velocity*/

/* apply control 
* all of this math is from Probabilistic Robotics by Thrun et al., 2006

having translational velocity and rotational velocity
using both to determine the next location
in the ideal case, the robot moves on a circle with radius r
(ideal as in there is no noise in the movement)
$v = \omega\cdot r$
$x_{t-1} = (x, y, \theta)^\top$
the center of the circle is at:

$\left(x - \frac{v}{\omega}\sin\theta, y + \frac{v}{\omega}\cos\theta\right)^\top$

after $\Delta t$ motion, the robot is at:


$\left(\begin{matrix}x'\\y'\\\theta'\end{matrix}\right) = \left(\begin{matrix}x_c + \frac{v}{\omega}\sin\theta \\ y_c - \frac{v}{\omega}\cos\theta \\ \Delta t\omega\end{matrix}\right)$


true motion is noisy and given by:

$\left(\begin{matrix}\hat{v}\\\hat{\omega}\end{matrix}\right) = \left(\begin{matrix}v\\\omega\end{matrix}\right) + \left(\begin{matrix}\epsilon_{\alpha_1v^2+\alpha_2\omega^2}\\\epsilon_{\alpha_3v^2+\alpha_4\omega^2}\end{matrix}\right)$

where $\epsilon_b \sim \text{Triangle}(0, b)$
$\alpha_n$ is an accuracy parameter that measures the error of the robot. the larger these values, the less accurate the robot
*/

/*
sample a random pose $x_t\sim p(x_t\mid u_t, x_{t-1})$
*/
Pose Robot::sample_xt(Control u, Pose p) {
    // sample noise to add to movement
    double mu_v = ALPHA_1 * pow(u.v, 2) + ALPHA_2 * pow(u.w, 2);
    double mu_w = ALPHA_3 * pow(u.v, 2) + ALPHA_4 * pow(u.w, 2);
    double v_eps = sample_triangular_dist(-1.0, mu_v, 1.0);
    double w_eps = sample_triangular_dist(-1.0, mu_w, 1.0);
    // add noise to movement
    double v_hat = u.v + v_eps;
    double w_hat = u.w + w_eps;
    // add noise to theta
    double g_hat = sample_triangular_dist(-1.0, (ALPHA_5 * pow(u.v, 2) + ALPHA_6 * pow(u.w, 2)), 1.0);
    // calculate next position
    double new_x = p.position.x() - ((v_hat / w_hat) * sin(p.orientation)) + ((v_hat / w_hat) * sin(p.orientation + w_hat * DT));
    double new_y = p.position.x() - ((v_hat / w_hat) * cos(p.orientation)) + ((v_hat / w_hat) * cos(p.orientation + w_hat * DT));
    double new_theta = p.orientation + (w_hat * DT) + (g_hat * DT);
    return Pose(Eigen::Vector2d({new_x, new_y}), new_theta);
}

/*
sets the robot's pose to the input pose, and adds a new trajectory event from `p` and `u`
*/
void Robot::move_to_new_pose(Pose p, Control u) {
    this->position = p.position;
    this->look_at = p.orientation;
    this->trajectory.push_back(make_traj_position(this->position, this->look_at, u.v, u.w));
}

double Robot::distance_to(Landmark landmark) {
    return abs((landmark.position - this->position).norm());
}

void Robot::look_to(Point2d point) {
    this->look_at = atan2(abs(point.y - this->position.y()), abs(point.x - this->position.x()));
}

void Robot::generate_lerp_trajectory(Point2d start, Point2d end, int n_steps) {
    /*
    uses lerp to generate a linear trajectory from `start` to `end`
    */
    Eigen::Vector2d start_pos(start.x, start.y);
    double total_dist = start.distance_to(end);
    double step_size = total_dist / n_steps;
    this->look_to(end);
    this->trajectory = {make_traj_position(start_pos, this->look_at, step_size, 0.0)};
    Eigen::Vector2d cur_pos = this->position;
    for (int i = 0; i < n_steps; i++) {
        Pose new_pose = this->sample_xt(Control(0.1, 0.0), Pose(cur_pos, this->look_at));
        this->trajectory.push_back(make_traj_position(new_pose.position, this->look_at, step_size, 0.0));
    }
}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << position.x() << ", " << position.y() << ") look_at: " << look_at << std::endl;
}

void Robot::write_traj_to_csv() {
    std::ofstream out("robot_trajectory.csv");
    // header? TODO
    out << "\"x\",\"y\",\"look_at\",\"v\",\"angular_v\"" << std::endl;
    for (const auto &t : this->trajectory) {
        out << t.first.position.x() << ',' << t.first.position.y() << ',' << t.first.orientation << ',' << t.second.v << ',' << t.second.w << std::endl;
    }
}