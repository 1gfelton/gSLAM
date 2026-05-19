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
// using namespace CONFIG;

// Init Methods
Robot::Robot() : position(0.0, 0.0), look_at(0.0) {}
Robot::Robot(const double _x, const double _y) : position(_x, _y), look_at(0.0) {}
Robot::Robot(const double _x, const double _y, const double _look_at) : position(_x, _y), look_at(_look_at) {}
// set position to the most recent point in the trajectory
Robot::Robot(const vector<pair<Pose, Control>> t) : trajectory(t), position(t[t.size() - 1].first.position), look_at(0.0) {}

/* apply control 
 all of this math is from Probabilistic Robotics by Thrun et al., 2006

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

$\left(\begin{matrix}\hat{v}\\\hat{\omega}\end{matrix}\right) = \left(\begin{matrix}v\\\omega\end{matrix}\right) + \left(\begin{matrix}\varepsilon_{\alpha_1v^2+\alpha_2\omega^2}\\\varepsilon_{\alpha_3v^2+\alpha_4\omega^2}\end{matrix}\right)$

where $\varepsilon_b \sim \text{Triangle}(0, b)$
$\alpha_n$ is an accuracy parameter that measures the error of the robot. the larger these values, the less accurate the robot
*/

/*
sample a random pose $x_t\sim p(x_t\mid u_t, x_{t-1})$
*/
Pose Robot::sample_xt(Control u, Pose p) {
    // sample noise to add to movement
    double var_v = (CONFIG::ALPHA_1 * pow(u.v, 2)) + (CONFIG::ALPHA_2 * pow(u.w, 2));
    double var_w = (CONFIG::ALPHA_3 * pow(u.v, 2)) + (CONFIG::ALPHA_4 * pow(u.w, 2));
    double v_eps = sample_triangular_dist(0.0, var_v);
    double w_eps = sample_triangular_dist(0.0, var_w);
    cout << "v_eps: " << v_eps << " w_eps: " << w_eps << std::endl;

    // add noise to movement
    double v_hat = u.v + v_eps;
    double w_hat = u.w + w_eps;

    // add noise to theta
    double var_g = ((CONFIG::ALPHA_5 * pow(u.v, 2)) + (CONFIG::ALPHA_6 * pow(u.w, 2)));
    double g_hat = sample_triangular_dist(0.0, var_g);

    // calculate next position
    double new_x, new_y;
    cout << "w_hat: " << w_hat << std::endl;
    // w == 0
    if (fabs(w_hat) < 1e-4) {
        new_x = p.position.x() - (v_hat * sin(to_radians(p.orientation))) + (v_hat * sin(to_radians(p.orientation + (w_hat * CONFIG::DT))));
        new_y = p.position.y() + (v_hat * cos(to_radians(p.orientation))) - (v_hat * cos(to_radians(p.orientation + (w_hat * CONFIG::DT))));
    } else {
        new_x = p.position.x() - ((v_hat / w_hat) * sin(to_radians(p.orientation))) + ((v_hat / w_hat) * sin(to_radians(p.orientation + (w_hat * CONFIG::DT))));
        new_y = p.position.y() + ((v_hat / w_hat) * cos(to_radians(p.orientation))) - ((v_hat / w_hat) * cos(to_radians(p.orientation + (w_hat * CONFIG::DT))));
    }
    double new_theta = p.orientation + (w_hat * CONFIG::DT) + (g_hat * CONFIG::DT);
    return Pose(Eigen::Vector2d(new_x, new_y), new_theta);
}

double Robot::get_motion_probability(Pose x, Control u, Pose prev) {
/*
`motion_model_velocity` from Thuring et al. Computes the probability of pose `x` given `u` and `prev`
*/
    double xx = prev.position.x();
    double x_prime = x.position.x();
    double yy = prev.position.y();
    double y_prime = x.position.y();

    double num = (xx - x_prime * cos(to_radians(this->look_at))) + ((yy - y_prime) * sin(to_radians(this->look_at)));
    double denom = ((yy - y_prime) * cos(to_radians(this->look_at))) - ((xx - x_prime) * sin(to_radians(this->look_at)));
    double mu = (1/2) * (num / denom);

    // $x^*$
    double x_ = ((xx + x_prime) / 2) + (mu * (yy - y_prime));
    // $y^*$
    double y_ = ((yy + y_prime) / 2) + (mu * (xx - x_prime));
    // $r^*$
    double r_ = sqrt(pow(xx - x_, 2) + pow(yy - y_, 2));
    // $\Delta\theta$
    double dtheta = atan2(y_prime - y_, x_prime - x_) - atan2(yy - y_, xx - x_);
    double dist = r_ * dtheta;
    double w_hat = (dtheta / CONFIG::DT);
    double v_hat = w_hat * r_;
    double g_hat = (((this->look_at + dtheta) - this->look_at) / CONFIG::DT) - w_hat;

    double var_v = (CONFIG::ALPHA_1 * pow(u.v, 2)) + (CONFIG::ALPHA_2 * pow(u.w, 2));
    double var_w = (CONFIG::ALPHA_3 * pow(u.v, 2)) + (CONFIG::ALPHA_4 * pow(u.w, 2));
    double var_g = (CONFIG::ALPHA_5 * pow(u.v, 2)) + (CONFIG::ALPHA_6 * pow(u.w, 2));
    return triangular_prob(u.v - v_hat, var_v) * triangular_prob(u.w - w_hat, var_w) * triangular_prob(g_hat, var_g);
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