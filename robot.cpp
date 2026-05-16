#include <iostream>
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


ideal motion is noisy and given by:

$\left(\begin{matrix}\hat{v}\\\hat{\omega}\end{matrix}\right) = \left(\begin{matrix}v\\\omega\end{matrix}\right) + \left(\begin{matrix}\epsilon_{\alpha_1v^2+\alpha_2\omega^2}\\\epsilon_{\alpha_3v^2+\alpha_4\omega^2}\end{matrix}\right)$

where $\epsilon_b \sim \text{Triangle}(0, b)$
$\alpha_n$ is an accuracy parameter that measures the error of the robot. the larger these values, the less accurate the robot
*/
void Robot::move_in_direction(Control u, Pose p) {
    /*
    move to `look_at` some distance `dist`
    */
    // TODO: Implement sample_motion_model_velocity algorithm
    // double v_hat = u.v + sample()
    double x_prime = p.position.x() + ((u.v / u.w) * sin(this->look_at));
    double y_prime = p.position.y() + ((u.v / u.w) * cos(this->look_at));
    double theta_prime = DT * u.w;

    double v_eps = 
    // Eigen::Vector2d delta = {cosd(this->look_at), sind(this->look_at)};
    // this->position += (dist * delta);

    // // now add this new direction to the trajectory
    // this->trajectory.push_back(make_traj_position(this->position, this->look_at, dist, 0.0));
}

float Robot::distance_to(Landmark landmark) {
    return abs((landmark.position - this->position).norm());
}

void Robot::generate_lerp_trajectory(Point2d start, Point2d end, int n_steps) {
    /*
    uses lerp to generate a linear trajectory from `start` to `end`
    */
    Eigen::Vector2d start_pos(start.x, start.y);
    double total_dist = start.distance_to(end);
    double step_size = total_dist / n_steps;
    this->trajectory = {make_traj_position(start_pos, this->look_at, step_size, 0.0)};
    Eigen::Vector2d cur_pos = this->position;
    for (int i = 0; i < n_steps; i++) {
        move_in_direction(step_size);
    }
}

// Print utility
void Robot::print() const {
    cout << "Robot at: (" << position.x() << ", " << position.y() << ") look_at: " << look_at << std::endl;
}