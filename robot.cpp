#include <iostream>
#include <stdexcept>
#include <fstream>
#include <math.h>
#include <spdlog/spdlog.h>
#include <float.h>
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

/* State vector, defined in block notation as $[\mathbf{x}_t, \mathbf{z}_{j:N}]\in\mathbb{R}^{3N+3\times1}$ 
Essentially you have the robot pose `[0:2]` and all of the features `[3:N*3]`
updates `Robot->state_vec` to be a `VectorXd`
*/
void Robot::update_state_vec(MatrixXd features) {
    // N + 3
    VectorXd state = VectorXd::Zero(features.size() + 3);
    // cout << "[robot.cpp]Robot location: " << this->position.x() << ", " << this->position.y() << std::endl;
    double x = this->position.x();
    double y = this->position.y();
    SPDLOG_INFO("Robot location: ({}, {})", x, y);
    // init x, y, theta
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
VectorXd Robot::sense_env(MatrixXd landmarks, int t) {
    // return the landmarks + some sensor noise
    // 3 x N
    MatrixXd features(3, N_LANDMARKS);
    for (int i = 0; i < N_LANDMARKS; i++) {
        cout << "Landmark:\tx: " << landmarks(0, i) << ",y: " << landmarks(1, i) << std::endl;
        this->observations[i].push_back(t); // mark at which timestep feature was observed
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
    /* TODO: uncomment noise from sensor reading */
    // 3 x N
    // MatrixXd noise = Eigen::MatrixXd::Random(features.rows(), features.cols());
    // MatrixXd ans = features + noise;
    // this->update_state_vec(ans);
    // convert the matrix 3 x N to vector of size 3N
    // VectorXd v = Eigen::Map<Eigen::VectorXd>(ans.data(), ans.size());
    // return v;
    this->update_state_vec(features);
    SPDLOG_INFO("Features:\n{}, {}", to_str(features), shape(features));
    return features.reshaped(features.size(), 1);
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
    if (isclose(w_hat, 0.0)) {
        new_x = p.position.x() + v_hat * sin(p.orientation) * DT;
        new_y = p.position.y() - v_hat * cos(p.orientation) * DT;
    } else {
        new_x = p.position.x() - ((v_hat / w_hat) * sin((p.orientation))) + ((v_hat / w_hat) * sin((p.orientation + (w_hat * DT))));
        new_y = p.position.y() + ((v_hat / w_hat) * cos((p.orientation))) - ((v_hat / w_hat) * cos((p.orientation + (w_hat * DT))));
    }
    double new_theta = p.orientation + (w_hat * DT) + (g_hat * DT);
    Vector3d res({new_x, new_y, new_theta});
    return res;
}

/*
calculates an ideal (noise-free) next pose (helper to replace re-computation of Thrun et al. equation (5.9))
*/
Vector3d Robot::get_next_pose(Vector3d mu_p, Vector2d u) {
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%l] [%s:%#] %v");

    double v = u[0];
    double w = u[1];
    double theta = mu_p[2];
    double v1, v2;
    // check for 0 angular velocity
    if (isclose(w, 0.0)) {
        v1 = v * cos(theta) * DT;
        v2 = v * -sin(theta) * DT;
    } else {
        v1 = (-v/w * sin(theta)) + (v/w * sin(theta + w * DT));
        v2 = (v/w * cos(theta)) - (v/w * cos(theta + w * DT));
    }
    double v3 = w * DT;
    Vector3d update({v1, v2, v3});
    return mu_p + update;
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

    // $3\times 3N + 3$
    MatrixXd Fx = MatrixXd::Zero(3, 3 * N_LANDMARKS + 3);
    // $[\mathbf{I}_{3\times3},0_{3\times3N}]$
    Fx.topLeftCorner(3, 3).setIdentity();

    // calculate noise-free next pose
    double v = u_t[0];
    double w = u_t[1];
    double theta = mu_p[2];
    double v1, v2;
    // check for 0 angular velocity
    if (isclose(w, 0.0)) {
        v1 = v * cos(theta) * DT;
        v2 = v * -sin(theta) * DT;
    } else {
        v1 = (-v/w * sin(theta)) + (v/w * sin(theta + w * DT));
        v2 = (v/w * cos(theta)) - (v/w * cos(theta + w * DT));
    }
    double v3 = w * DT;
    Vector3d update({v1, v2, v3});



    // $\bar{\mu}_t = \mu_{t-1} + F_x^T \begin{pmatrix} -\dfrac{v_t}{\omega_t} \sin\mu_{t-1,\theta} + \dfrac{v_t}{\omega_t} \sin(\mu_{t-1,\theta} + \omega_t \Delta t) \\[8pt] \dfrac{v_t}{\omega_t} \cos\mu_{t-1,\theta} - \dfrac{v_t}{\omega_t} \cos(\mu_{t-1,\theta} + \omega_t \Delta t) \\[8pt] \omega_t \Delta t \end{pmatrix}$



    //               3N+3 x 1     3N+3 x 3         3 x 1
    VectorXd mu_bar = mu_p + Fx.transpose() * update; // 3N+3 x 1

    // $G_t = I + F_x^\top g_t F_x$
    MatrixXd I = MatrixXd::Identity(3 * N_LANDMARKS + 3, 3 * N_LANDMARKS + 3);
    MatrixXd g_t = MatrixXd::Zero(3, 3);
    Vector2d tmp = {-v2, v1};
    g_t.block(0, 2, 2, 1) = tmp;
    MatrixXd Gt = I + Fx.transpose() * g_t * Fx; // 3N+3 x 3N+3

    // $\bar{\Sigma}_t = G_t\Sigma_{t-1}G_t^\top + F_x^\top R_t F_x$
    MatrixXd cov_bar = (Gt * cov_p * Gt.transpose()) + (Fx.transpose() * Rt * Fx); // 3N+3 x 3N+3

    MatrixXd Qt = MatrixXd::Identity(3, 3);
    Qt.diagonal() = Vector3d{SIGMA_R, SIGMA_PHI, SIGMA_S};
    // for feature i, we get landmark j from the correspondence value
    for (int i = 0; i < N_LANDMARKS; i++) {
        // get the correspondence
        int j = c_t(i) * 3;
        double theta = mu_bar(2);
        // handle landmarks that haven't yet been seen
        bool not_seen = (isclose((double)mu_bar(j), 0.0) && 
                        isclose((double)mu_bar(j + 1), 0.0) && 
                        isclose((double)mu_bar(j + 2), 0.0));
        if (not_seen == true) {
            double theta = mu_bar(2);
            double phi = z_t(i * 3 + 1);
            double r = z_t(i * 3);
            mu_bar(j) = mu_bar(0) + r * cos(phi + theta); // x
            mu_bar(j + 1) = mu_bar(1) + r * sin(phi + theta); // y
            mu_bar(j + 2) = z_t(i * 3 + 2); // signature
        }

        Vector2d d = mu_bar(seq(j, j + 1)) - mu_bar(seq(0, 1));
        double q = d.squaredNorm();

        if (isclose(q, 0.0)) {
            throw std::runtime_error("q is zero");
        }

        Vector3d z_hat{sqrt(q), atan2(d(1), d(0)) - theta, mu_bar(j + 2)};
        MatrixXd Fxj = MatrixXd::Zero(6, 3 * N_LANDMARKS + 3);
        Fxj.block(0, 0, 3, 3).setIdentity();
        Fxj.block(3, j, 3, 3).setIdentity();
        MatrixXd tmp(3, 6);
        tmp << -sqrt(q) * d.x(), -sqrt(q) * d.y(), 0, sqrt(q) * d.x(), sqrt(q) * d.y(), 0, d.y(), -d.x(), -q, -d.y(), d.x(), 0, 0, 0, 0, 0, 0, q;

        MatrixXd Ht = (1 / q) * tmp * Fxj; // 3 x 3N + 3
        // $K^i_t = \bar{\Sigma}_tH_t^{i\top}(H^i_t\bar{\Sigma}_tH_t^{i\top}+Q_t)^{-1}$

        //          3N+3x3N+3    3N+3x3         3x3N+3  3N+3x3N+3  3N+3x3        3x3 
        MatrixXd K = cov_bar * Ht.transpose() * (Ht * cov_bar * Ht.transpose() + Qt).inverse(); // 3N+3 x 3
        //    3N+3x3   3N+3   3N+3
        mu_bar += K * (z_t(seq(i * 3, i * 3 + 2)) - z_hat);
        cov_bar = (MatrixXd::Identity(K.rows(), Ht.cols()) - K * Ht) * cov_bar;
    }
    // update $\mu_t = \bar{\mu}_t$ and $\Sigma_t = \bar{\Sigma}_t$
    this->state_vec = mu_bar;
    this->covariance = cov_bar;
}

std::pair<MatrixXd, MatrixXd> Robot::Graph_SLAM_solve(MatrixXd omega_, MatrixXd xi_, MatrixXd omega, MatrixXd xi) {
    MatrixXd sigma = omega_.inverse();
    // VectorXd mu_ = sigma * xi_;
    VectorXd mu = VectorXd::Zero(N_STEPS * 3 + N_LANDMARKS * 3);
    mu.head(N_STEPS * 3) = sigma * xi_;

    MatrixXd omega_inv = omega.inverse();
    for (int j = 0; j < N_LANDMARKS; j++) {
        VectorXi tau = Eigen::Map<VectorXi>(this->observations[j].data(), this->observations[j].size());
        tau *= 3; 
        int jx = (j * 3) + (N_STEPS * 3);
        SPDLOG_INFO("mu:\n{}", to_str(mu));
        SPDLOG_INFO("tau:\n{}, {}", to_str(tau), shape(tau));
        if (tau.size()) {
            for (const auto &pose : tau) {
                SPDLOG_INFO("mu block:\n{}", to_str(mu.block(jx, 0, 3, 1)));
                mu.block(jx, 0, 3, 1) = omega_inv(jx, jx) * (xi.block(jx, 0, 3, 1) + omega.block(jx, pose, 3, 3) * mu.block(pose, 0, 3, 1));
                SPDLOG_INFO("mu block after:\n{}", to_str(mu.block(jx, 0, 3, 1)));
            }
        }
    }
    SPDLOG_INFO("Finished solving...");
    return std::make_pair(mu, sigma);
}

/*
`omega` : Information matrix
`xi` : Information vector
Algorithm Graph_SLAM_reduce from Thrun et al. 2006
*/
std::pair<MatrixXd, MatrixXd> Robot::Graph_SLAM_reduce(MatrixXd omega, MatrixXd xi) {
    MatrixXd omega_ = omega;
    MatrixXd xi_ = xi;
    SPDLOG_INFO("state_vec:\n{}", to_str(this->state_vec));
    MatrixXd state = this->state_vec.reshaped(this->state_vec.size()/3, 3);
    MatrixXd xi_view = xi_.reshaped(xi_.size() / 3, 3);
    SPDLOG_INFO("xi_view:\n{}", to_str(xi_view));
    SPDLOG_INFO("State:\n{}", to_str(state));
    for (int j = 0; j < (this->state_vec.size() - 3) / 3; j++) {
        // $\tau(j)$ is the set of all poses at which $j$ was observed
        VectorXi tau = Eigen::Map<VectorXi>(this->observations[j].data(), this->observations[j].size());
        tau *= 3;
        SPDLOG_INFO("tauj:\n{}", to_str(tau));
        int jx = N_STEPS * 3 + j * 3;

        for (const auto &a : tau) {
            MatrixXd d1 = omega_.block(a, jx, 3, 3) * omega_.inverse().block(jx, jx, 3, 3) * xi_.block(a, 0, 3, 1);
            SPDLOG_INFO("d1:\n{}, {}", to_str(d1), shape(d1));
            xi_view(tau, Eigen::all) -= d1.transpose();
            SPDLOG_INFO("xi_view:\n{}", to_str(xi_view));
            for (const auto &b : tau) {
                auto d2 = omega_.block(a, jx, 3, 3) * omega_.inverse().block(jx, jx, 3, 3) * omega_.block(jx, b, 3, 3);
                SPDLOG_INFO("d2:\n{}", to_str(d2));
                omega_.block(a, b, 3, 3) -= d2;
                SPDLOG_INFO("omega_:\n{}", to_str(omega_));
            }
        }
        SPDLOG_INFO("omega_:\n{}", to_str(omega_));
    }
    // remove all landmarks from omega, xi 
    MatrixXd final_omega = omega_.block(0, 0, N_STEPS * 3, N_STEPS * 3);
    SPDLOG_INFO("final omega:\n{}", to_str(final_omega));
    MatrixXd final_xi = xi_.block(0, 0, N_STEPS * 3, 1);
    SPDLOG_INFO("final xi:\n{}", to_str(final_xi));
    return std::make_pair(final_omega, final_xi);
}

/*
`u` : Sequence of controls
`z` : Set of measurements (`vector<VectorXd>` where each `VectorXd` is a measurement at timestep `t`)
`c` : Sequence of correspondences
`mu` : State vector
*/
std::pair<MatrixXd, MatrixXd> Robot::Graph_SLAM_linearize(VectorXd u, vector<VectorXd> z, VectorXi c, VectorXd mu) {
    spdlog::set_pattern("[%l] [%s:%#] %v");

    Matrix3d R = Matrix3d(Vector3d({SIGMA_X, SIGMA_Y, SIGMA_THETA}).asDiagonal());
    SPDLOG_INFO("R:\n{}", to_str(R));
    MatrixXd Ri = R.inverse();
    double INF = 10e10;
    MatrixXd omega = MatrixXd(Vector3d::Constant(INF).asDiagonal());
    MatrixXd xi = MatrixXd::Zero(3, 1);
    // update positions based on controls
    for (int t = 0; t < N_STEPS - 1; t++) { 
        // int i = t * 2; // indexing into controls
        Vector2d cur_control = u(seq(t*2, t*2 + 1));
        Vector3d cur_pose = mu(seq(t*3, t*3 + 2));
        Vector3d x_hat = this->get_next_pose(cur_pose, cur_control);
        MatrixXd G = MatrixXd::Identity(3, 3);
        G.topRightCorner(2, 1) = x_hat.head(2); // TODO

        // $\Omega_{x_t, x_{t-1}} \mathrel{+}= \begin{pmatrix}-G^\top_t \\ 1\end{pmatrix}R_t^{-1}(G_t\space\space1)$

        MatrixXd G_tmp = MatrixXd::Zero(6, 3);
        G_tmp.block(0, 0, 3, 3) = -G.transpose();
        G_tmp.block(3, 0, 3, 3).setIdentity();

        MatrixXd res = G_tmp * Ri * G_tmp.transpose(); // 6 x 6
        omega.conservativeResize(omega.rows() + 3, omega.cols() + 3);
        // update t and t - 1
        omega.block(t*3, t*3, 6, 6) += res;

        // $\xi_{x_t, x_{t-1}} \mathrel{+}= \begin{pmatrix}-G^\top_t \\ 1\end{pmatrix}R_t^{-1}[\hat{x}_t - G_t\mu_{t-1}]$

        MatrixXd res1 = G_tmp * Ri * (x_hat - G * cur_pose); // 6 x 1
        xi.conservativeResize(xi.rows() + 3, xi.cols());
        // update t & t - 1
        xi.block(t*3, 0, 6, 1) += res1;
        SPDLOG_INFO("xi:\n{}", to_str(xi));
    }
    SPDLOG_INFO("omega after controls:\n{}", to_str(omega));
    // resize omega to include spaces for map
    omega.conservativeResize(omega.rows() + c.size() * 3, omega.cols() + c.size() * 3);
    xi.conservativeResize(xi.rows() + c.size() * 3, xi.cols());
    SPDLOG_INFO("resized omega:\n{}", to_str(omega));
    // assume z contains T many measurements and each measurement has N many features
    // TODO: some annoying issues - since correspondences are 1 indexed, to correctly index into them we need to do (j - 1) * 3 + (N_STEPS * 3)
    // this creates some frustrating/annoying bugs that are tricky to debug 
    MatrixXd Q_inv = MatrixXd(Vector3d({SIGMA_R, SIGMA_PHI, SIGMA_S}).asDiagonal()).inverse();
    for (int t = 0; t < z.size(); t++) {
        // SPDLOG_INFO("Q:\n{}", to_str(Q));
        for (int i = 0; i < c.size(); i++) {
            int j = c(i);
            // must do *3 as there as mu is a vector of triplets (x, y, theta)
            // update omega
            Vector2d d = mu(seq(j*3, j*3 + 1)) - mu(seq(0, 1));
            double q = d.squaredNorm();
            if (isclose(q, 0.0)) {
                throw std::runtime_error("q is zero - This means mu_j == mu_0 so something is wrong with your state vector.");
            }

            Vector3d z_hat{sqrt(q), atan2(d(1), d(0)) - mu(2), mu(j*3 + 2)};
            MatrixXd tmp(3, 6);
            tmp << -sqrt(q) * d.x(), -sqrt(q) * d.y(), 0, sqrt(q) * d.x(), sqrt(q) * d.y(), 0, d.y(), -d.x(), -q, -d.y(), d.x(), 0, 0, 0, 0, 0, 0, q;
            MatrixXd H = (1 / q) * tmp; // 3 x 6
            SPDLOG_INFO("H:\n{}", to_str(H));
            // add $H_t^{i\top}Q^{-1}_tH_t^i$ to $\Omega$ at $x_t, m_j$
            MatrixXd HH = H.transpose() * Q_inv * H;
            Matrix3d A = HH.block(0, 0, 3, 3);
            Matrix3d B = HH.block(0, 3, 3, 3);
            Matrix3d C = HH.block(3, 3, 3, 3);
            omega.block(t*3, t*3, 3, 3) += A; // $x_t$
            omega.block(t*3, (j-1)*3 + (N_STEPS * 3), 3, 3) += B; // $x_t$
            omega.block((j-1)*3 + (N_STEPS * 3), (j - 1)*3 + (N_STEPS * 3), 3, 3) += C; // $m_j$
            omega.block((j-1)*3 + (N_STEPS * 3), t*3, 3, 3) += B.transpose(); // $m_j$

            // update xi
            MatrixXd z_t = z[t](seq(i*3, i*3 + 2));
            MatrixXd tmpv(6, 1); tmpv <<  mu(0), mu(1), mu(2), mu(j*3), mu(j*3 + 1), mu(j*3 + 2);
            MatrixXd HZ = H.transpose() * Q_inv * (z_t - z_hat + (H * tmpv));
            SPDLOG_INFO("HZ:\n{}", to_str(HZ));
            SPDLOG_INFO("xi before:\n{}", to_str(xi));
            xi.block(t*3, 0, 3, 1) += HZ.block(0, 0, 3, 1);
            SPDLOG_INFO("First ok");
            xi.block((j-1)*3 + (N_STEPS*3), 0, 3, 1) += HZ.block(3, 0, 3, 1);
            SPDLOG_INFO("Second ok");
            SPDLOG_INFO("xi after:\n{}", to_str(xi));
        }
        SPDLOG_INFO("final omega:\n{}", to_str(omega));
    }
    return std::make_pair(omega, xi); // omega: 3N + N_LANDMARKS x 3N + N_LANDMARKS
}

/*
`u_t` - a sequence of controls from timestep 0 : t
returns `mu`, a `VectorXd` of triplets where each triplet corresponds to a pose at `x_t`
*/
VectorXd Robot::Graph_SLAM_init(VectorXd u_t) {
    VectorXd mu = Vector3d::Zero();
    Vector3d prev = mu;
    // for each control, use the previous pose to find next pose
    for (int i = 0; i < u_t.size(); i+=2) {
        Vector2d u = u_t(seq(i, i+1));
        Vector3d mu_t = this->get_next_pose(prev, u);
        // append this new pose to the state vector
        mu.conservativeResize(mu.size() + 3);
        mu.tail(3) = mu_t;
        prev = mu_t;
    }
    return mu;
}

VectorXd Robot::Graph_SLAM(VectorXd u, vector<VectorXd> z_t, VectorXi c_t) {
    VectorXd mu = this->Graph_SLAM_init(u);
    bool convergence = false;
    while (convergence == false) {
        auto [omega, xi] = this->Graph_SLAM_linearize(u, z_t, c_t, mu);
        auto [omega_, xi_] = this->Graph_SLAM_reduce(omega, xi);
        // auto [mu_new, Sigma] = this->Graph_SLAM_solve(omega_, xi_, omega, xi);
        // mu = mu_new;
    }
    return mu;
}

double Robot::get_motion_probability(Pose x, Control u, Pose prev) {
/*
`motion_model_velocity` from Thrun et al. Computes the probability of pose `x` given `u` and `prev`
*/
    double xx = prev.position.x();
    double x_prime = x.position.x();
    double yy = prev.position.y();
    double y_prime = x.position.y();

    double num = (xx - x_prime * cos((this->look_at))) + ((yy - y_prime) * sin((this->look_at)));
    double denom = ((yy - y_prime) * cos((this->look_at))) - ((xx - x_prime) * sin((this->look_at)));
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