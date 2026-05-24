#include <iostream>
#include <stdexcept>
#include <fstream>
#include <math.h>
#include "robot.h"
#include "point2d.h"
#include "utils.h"
#include "control.h"
#include "pose.h"
#include "config.h"

using namespace Eigen;
using namespace CONFIG;

using std::cout;
using std::vector;
using std::pair;

// Init Methods
Robot::Robot() : position(0.0, 0.0), look_at(0.0), covariance{MatrixXd::Zero(3 * N_LANDMARKS + 3, 3 * N_LANDMARKS + 3)} {}
Robot::Robot(const double _x, const double _y) : position(_x, _y), look_at(0.0), covariance{MatrixXd::Zero(3 * N_LANDMARKS + 3, 3 * N_LANDMARKS + 3)}{}
Robot::Robot(const double _x, const double _y, const double _look_at) : position(_x, _y), look_at(_look_at), covariance{MatrixXd::Zero(3 * N_LANDMARKS + 3, 3 * N_LANDMARKS + 3)}{}
// set position to the most recent point in the trajectory
Robot::Robot(const vector<pair<Pose, Control>> t) : trajectory(t), position(t[t.size() - 1].first.position), look_at(0.0), covariance{MatrixXd::Zero(3 * N_LANDMARKS + 3, 3 * N_LANDMARKS + 3)}{}

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

/* features: $\mathbf{f}_t \in \mathbb{R}^{3\times N}$ */
void Robot::update_state_vec(MatrixXd features) {
    // N + 3
    VectorXd state = VectorXd::Zero(features.size() + 3);
    cout << "[robot.cpp]Robot location: " << this->position.x() << ", " << this->position.y() << std::endl;
    state(0) = this->position.x(); state(1) = this->position.y(); state(2) = this->look_at;
    for (int i = 0; i < features.cols(); i++) {
        for (int j = 0; j < 3; j++) {
            state[i * 3 + j + 3] = features(j, i);
        }
    }
    this->state_vec = state;
}

/*
Simulates the robot taking a sensor reading of its environment. 
converts the landmarks to features and adds noise to them.
Gives the feature vec at current timestep $f_t(z_t)$
Updates the state vector of the robot
*/
VectorXd Robot::sense_env(MatrixXd landmarks) {
    // return the landmarks + some sensor noise
    // 3 x N
    MatrixXd features(3, N_LANDMARKS);
    for (int i = 0; i < N_LANDMARKS; i++) {
        cout << "Landmark:\tx: " << landmarks(0, i) << ",y: " << landmarks(1, i) << std::endl;
        double dx = landmarks(0, i) - this->position.x();
        double dy = landmarks(1, i) - this->position.y();

        double r = sqrt(dx*dx + dy*dy);
        double phi = atan2(dy, dx); // in radians
        double s = landmarks(0, i); // signature can be x value for now
        cout << "Feature:\tr: " << r << ",phi: " << phi << ",s: " << s << std::endl;

        features(0, i) = r;
        features(1, i) = phi;
        features(2, i) = s;
    }
    // 3 x N
    MatrixXd noise = Eigen::MatrixXd::Random(features.rows(), features.cols());
    MatrixXd ans = features + noise;
    this->update_state_vec(ans);
    // convert the matrix 3 x N to vector of size 3N
    VectorXd v = Eigen::Map<Eigen::VectorXd>(ans.data(), ans.size());
    return v;
}

/*
sample a random pose $x_t\sim p(x_t\mid u_t, x_{t-1})$
*/
Vector3d Robot::sample_xt(Control u, Pose p) {
    // sample noise to add to movement
    double var_v = (ALPHA_1 * pow(u.v, 2)) + (ALPHA_2 * pow(u.w, 2));
    double var_w = (ALPHA_3 * pow(u.v, 2)) + (ALPHA_4 * pow(u.w, 2));
    double v_eps = sample_triangular_dist(0.0, var_v);
    double w_eps = sample_triangular_dist(0.0, var_w);
    cout << "v_eps: " << v_eps << " w_eps: " << w_eps << std::endl;

    // add noise to movement
    double v_hat = u.v + v_eps;
    double w_hat = u.w + w_eps;

    // add noise to theta
    double var_g = ((ALPHA_5 * pow(u.v, 2)) + (ALPHA_6 * pow(u.w, 2)));
    double g_hat = sample_triangular_dist(0.0, var_g);

    // calculate next position
    double new_x, new_y;
    cout << "w_hat: " << w_hat << std::endl;
    // w == 0
    if (fabs(w_hat) < 1e-4) {
        new_x = p.position.x() - (v_hat * sin(to_radians(p.orientation))) + (v_hat * sin(to_radians(p.orientation + (w_hat * DT))));
        new_y = p.position.y() + (v_hat * cos(to_radians(p.orientation))) - (v_hat * cos(to_radians(p.orientation + (w_hat * DT))));
    } else {
        new_x = p.position.x() - ((v_hat / w_hat) * sin(to_radians(p.orientation))) + ((v_hat / w_hat) * sin(to_radians(p.orientation + (w_hat * DT))));
        new_y = p.position.y() + ((v_hat / w_hat) * cos(to_radians(p.orientation))) - ((v_hat / w_hat) * cos(to_radians(p.orientation + (w_hat * DT))));
    }
    double new_theta = p.orientation + (w_hat * DT) + (g_hat * DT);
    Vector3d res({new_x, new_y, new_theta});
    return res;
}

/*
EKF SLAM per Thrun et al. 2006
mu_p: $\mu_{t-1}$ is the state vector: $[x, y, \theta, m_{1,x}, m_{1,y}, s_1, \cdots, m_{N,x}, ,m_{N,y}, s_N]^\top$
cov_p: $\Sigma_{t-1}$ is the covariance matrix (3N + 3 x 3N + 3)
u_t: $u_t$ is the control at current timestep
z_t: $z_t$ are the features at current timestep (ranges, bearings, signature) $z_t\in\mathbb{R}^{3\times N}$
c_t: $c_t$ are the known correspondences at current timestep $c_t^i\in\{1,\cdots,N+1\}$
*/
void Robot::EKF_SLAM(VectorXd mu_p, MatrixXd cov_p, Vector2d u_t, VectorXd z_t, VectorXi c_t) {
    MatrixXd Rt = MatrixXd::Identity(3, 3);
    Rt.diagonal() = Vector3d{SIGMA_X, SIGMA_Y, SIGMA_THETA};
    cout << "R:\n" << Rt << std::endl;

    // $3\times 3N + 3$
    MatrixXd Fx = MatrixXd::Zero(3, 3 * N_LANDMARKS + 3);
    // $[\mathbf{I}_{3\times3},0_{3\times3N}]$
    Fx.topLeftCorner(3, 3).setIdentity();

    // calculate noise-free next pose
    double frac = (u_t[0] / u_t[1]);
    double w = u_t[1];
    double theta = mu_p[2];
    double v1 = (-frac * sin(theta)) + (frac * sin(theta + w * DT));
    double v2 = (frac * cos(theta)) - (frac * sin(theta + w * DT));
    double v3 = w * DT;
    Vector3d update({v1, v2, v3});



    // $\bar{\mu}_t = \mu_{t-1} + F_x^T \begin{pmatrix} -\dfrac{v_t}{\omega_t} \sin\mu_{t-1,\theta} + \dfrac{v_t}{\omega_t} \sin(\mu_{t-1,\theta} + \omega_t \Delta t) \\[8pt] \dfrac{v_t}{\omega_t} \cos\mu_{t-1,\theta} - \dfrac{v_t}{\omega_t} \cos(\mu_{t-1,\theta} + \omega_t \Delta t) \\[8pt] \omega_t \Delta t \end{pmatrix}$



    //               3N+3 x 1     3N+3 x 3         3 x 1
    VectorXd mu_bar = mu_p + Fx.transpose() * update; // 3N+3 x 1

    // $G_t = I + F_x^\top g_t F_x$
    MatrixXd I = MatrixXd::Identity(3 * N_LANDMARKS + 3, 3 * N_LANDMARKS + 3);
    MatrixXd g_t = MatrixXd::Zero(3, 3);
    cout << "v1: " << v1 << " v2: " << v2 << std::endl;;
    Vector2d tmp = {-v2, v1};
    g_t.block(0, 2, 2, 1) = tmp;
    MatrixXd Gt = I + Fx.transpose() * g_t * Fx; // 3N+3 x 3N+3
    cout << "Gt:\n" << Gt << std::endl;

    // $\bar{\Sigma}_t = G_t\Sigma_{t-1}G_t^\top + F_x^\top R_t F_x$
    MatrixXd cov_bar = (Gt * cov_p * Gt.transpose()) + (Fx.transpose() * Rt * Fx); // 3N+3 x 3N+3
    cout << "cov_bar:\n" << cov_bar << std::endl;

    MatrixXd Qt = MatrixXd::Identity(3, 3);
    Qt.diagonal() = Vector3d{SIGMA_R, SIGMA_PHI, SIGMA_S};
    cout << "Q:\n" << Qt << std::endl;

    for (int i = 1; i < N_LANDMARKS; i++) {
        // get the correspondence
        int j = c_t(i);
        cout << "c: " << j << std::endl;
        int ii = j * 3; // mu_bar \in R^3N
        double theta = mu_bar(2);
        // handle landmarks that haven't yet been seen
        bool not_seen = ((double)isclose(mu_bar(i * 3), 0.0) && isclose((double)mu_bar(i * 3 + 1), 0.0) && isclose((double)mu_bar(i * 3 + 2), 0.0));
        if (not_seen == true) {
            cout << "Haven't seen this feature before!" << std::endl;
            double theta = mu_bar(2);
            cout << "Theta: " << theta << std::endl;
            double phi = z_t(ii + 1);
            cout << "PHI: " << phi << std::endl;
            double r = z_t(ii);
            cout << "r: " << r << std::endl;
            mu_bar(ii) = mu_bar(0) + r * cos(phi + theta); // x
            cout << "x: " << mu_bar(0) << std::endl;
            mu_bar(ii + 1) = mu_bar(1) + r * sin(phi + theta); // y
            cout << "y: " << mu_bar(1) << std::endl;
            mu_bar(ii + 2) = z_t(ii + 2); // signature
            cout << "sig: " << z_t(ii + 2) << std::endl;
        }

        Vector2d d = mu_bar(seq(ii, ii + 1)) - mu_bar(seq(0, 1));
        cout << "d:\n" << d << std::endl;
        double q = d.squaredNorm();
        cout << "d:\n" << d << std::endl;
        cout << "q:\n" << q << std::endl;

        if (isclose(q, 0.0)) {
            throw std::runtime_error("q is zero");
        }

        Vector3d z_hat{sqrt(q), atan2(d(1), d(0)) - theta, mu_bar(i * 3 + 2)};
        MatrixXd Fxj = MatrixXd::Zero(6, 3 * N_LANDMARKS + 3);
        Fxj.block(0, 0, 3, 3).setIdentity();
        Fxj.block(3, 3 * j, 3, 3).setIdentity();
        cout << "Fxj:\n" << Fxj << std::endl;
        MatrixXd tmp(3, 6);
        tmp << -sqrt(q) * d.x(), -sqrt(q) * d.y(), 0, sqrt(q) * d.x(), sqrt(q) * d.y(), 0, d.y(), -d.x(), -q, -d.y(), d.x(), 0, 0, 0, 0, 0, 0, q;
        cout << "tmp:\n" << tmp << std::endl;

        MatrixXd Ht = (1 / q) * tmp * Fxj; // 3 x 3N + 3
        cout << "Ht:\n" << Ht << std::endl;
        // $K^i_t = \bar{\Sigma}_tH_t^{i\top}(H^i_t\bar{\Sigma}_tH_t^{i\top}+Q_t)^{-1}$

        // TODO: figure out shapes from matmul
        //          3N+3x3N+3    3N+3x3         3x3N+3  3N+3x3N+3  3N+3x3        3x3 
        MatrixXd K = cov_bar * Ht.transpose() * (Ht * cov_bar * Ht.transpose() + Qt).inverse(); // 3N+3 x 3
        cout << "K:\n" << K << std::endl; 
        //    3N+3x3   3N+3   3N+3
        mu_bar += K * (z_t(seq(i*3, i*3 + 2)) - z_hat);
        cout << "mu_bar:\n" << mu_bar << std::endl;
        cov_bar = (MatrixXd::Identity(K.rows(), Ht.cols()) - K * Ht) * cov_bar;
    }
    // update $\mu_t = \bar{\mu}_t$ and $\Sigma_t = \bar{\Sigma}_t$
    this->state_vec = mu_bar;
    this->covariance = cov_bar;
}

double Robot::get_motion_probability(Pose x, Control u, Pose prev) {
/*
`motion_model_velocity` from Thrun et al. Computes the probability of pose `x` given `u` and `prev`
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
    double w_hat = (dtheta / DT);
    double v_hat = w_hat * r_;
    double g_hat = (((this->look_at + dtheta) - this->look_at) / DT) - w_hat;

    double var_v = (ALPHA_1 * pow(u.v, 2)) + (ALPHA_2 * pow(u.w, 2));
    double var_w = (ALPHA_3 * pow(u.v, 2)) + (ALPHA_4 * pow(u.w, 2));
    double var_g = (ALPHA_5 * pow(u.v, 2)) + (ALPHA_6 * pow(u.w, 2));
    return triangular_prob(u.v - v_hat, var_v) * triangular_prob(u.w - w_hat, var_w) * triangular_prob(g_hat, var_g);
}

/*
sets the robot's pose to the input pose, and adds a new trajectory event from `p` and `u`
*/
void Robot::move_to_new_pose(Pose p, Control u) {
    this->position = p.position;
    this->look_at = p.orientation;
    this->trajectory.push_back(make_traj_position(this->position.head<2>(), this->look_at, u.v, u.w));
}

double Robot::distance_to(Landmark landmark) {
    return abs((landmark.position - this->position).norm());
}

void Robot::look_to(Point2d point) {
    this->look_at = atan2(abs(point.position.y() - this->position.y()), abs(point.position.x() - this->position.x()));
}

void Robot::generate_lerp_trajectory(Point2d start, Point2d end, int n_steps) {
    /*
    uses lerp to generate a linear trajectory from `start` to `end`
    */
    Vector2d start_pos(start.position.x(), start.position.y());
    double total_dist = start.distance_to(end);
    double step_size = total_dist / n_steps;
    this->look_to(end);
    this->trajectory = {make_traj_position(start_pos.head<2>(), this->look_at, step_size, 0.0)};
    Vector2d cur_pos = this->position;
    for (int i = 0; i < n_steps; i++) {
        Vector3d new_pose = this->sample_xt(Control(0.1, 0.0), Pose(cur_pos, this->look_at));
        this->trajectory.push_back(make_traj_position(new_pose.head<2>(), this->look_at, step_size, 0.0));
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