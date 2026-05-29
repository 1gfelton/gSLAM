#include "world.h"
#include "landmark.h"
#include "point2d.h"
#include "robot.h"
#include "utils.h"
#include "config.h"
#include <spdlog/spdlog.h>
#include <bits/stdc++.h>
#include <fstream>
#include <Eigen/Dense>
#include <fmt/core.h>

using namespace std;
using namespace Eigen;
using namespace CONFIG;

void P_test_sorting() {
    cout << "Testing sorting..." << endl;
    int n = 10;
    vector<Point2d> L(n);
    for (int i = 0; i < n; i++) {
        Point2d ll;
        ll.randomize(0.0, 1.0);
        L[i] = ll;
    }
    sort(L.begin(), L.end());
    cout << "Sorted Point2ds: \n";
    for (const auto &l : L) {
        l.print();
    }
    cout << "Reversed Point2ds: ";
    reverse(L.begin(), L.end());
    for (const auto &l : L) {
        l.print();
    }
    cout << "Passed!" << endl;
}

void P_test_eigen_assignment() {
    cout << "Testing Eigen vector assignment..." << endl;
    int n = 10;
    vector<Point2d> L(n);
    for (int i = 0; i < n; i++) {
        Point2d ll;
        ll.randomize(0.0, 1.0);
        L[i] = ll;
        ll.print();
        cout << "position: " << ll.position << '\n';
    }
    cout << "Passed!" << endl;
}

void P_test_similarity() { 
    cout << "Testing Similarity..." << endl;
    Point2d a(0.0, 0.0);
    Point2d b(0.0, 0.0);
    bool ans = true;
    bool res = (a == b);
    if (res != ans) {
        throw std::runtime_error("Failed similarity test.");
    }
    cout << "Passed!" << endl;
}

void LM_test_sorting() {
    cout << "Testing sorting..." << endl;
    int n = 10;
    vector<Landmark> L(n);
    for (int i = 0; i < n; i++) {
        Landmark ll;
        ll.randomize(0.0, 1.0);
        L[i] = ll;
    }
    sort(L.begin(), L.end());
    cout << "Sorted Landmarks: \n";
    for (const auto &l : L) {
        l.print();
    }
    cout << "Reversed Landmarks: ";
    reverse(L.begin(), L.end());
    for (const auto &l : L) {
        l.print();
    }
    cout << "Passed!" << endl;
}

void LM_test_eigen_assignment() {
    cout << "Testing Eigen vector assignment..." << endl;
    int n = 10;
    vector<Landmark> L(n);
    for (int i = 0; i < n; i++) {
        Landmark ll;
        ll.randomize(0.0, 1.0);
        L[i] = ll;
        ll.print();
        cout << "position: " << ll.position << '\n';
    }
    cout << "Passed!" << endl;
}

void LM_test_similarity() { 
    cout << "Testing Similarity..." << endl;
    Landmark a(0.0, 0.0);
    Landmark b(0.0, 0.0);
    bool ans = true;
    bool res = (a == b);
    if (res != ans) {
        throw std::runtime_error("Failed similarity test.");
    }
    cout << "Passed!" << endl;
}

void printj(Robot r) {
    cout << "Printing Trajectory...\n";
    for (const auto &t : r.trajectory) {
        t.first.print(); t.second.print();
        // t.print();
    }
    cout << endl;
}

void to_csv(VectorXd v, string file) {
    cout << "Writing to csv file " << file + ".csv..." << endl;
    string filename = "./csvs/" + file + ".csv";
    std::ofstream out(filename, std::ofstream::trunc);
    out << "\"x\",\"y\",\"theta\"\n";
    for (int i = 0; i < v.size(); i+=3) {
        out << v(i) << ',' << v(i + 1) << ',' << v(i + 2) << "\n";
    }
    out.close();
    cout << "Wrote " << v.size()/3 << " lines to " << file + ".csv..." << endl;
}

struct TestRobot {
    Robot r;
    TestRobot() {
        Point2d pos;
        pos.randomize(0.0, 1.0);
        // create random robot pos
        Robot _r(pos.position.x(), pos.position.y());
        r = _r;
        r.look_at = THETA;
    }

    void print_features(MatrixXd feats) {
        cout << "Features:\n";
        for (int i = 0; i < feats.cols(); i++) {
            cout << "r:\t" << feats(0, i) << "\tphi:\t" << feats(1, i) << "\ts:\t" << feats(2, i) << endl;
        }
    }

    void print_state() {
        cout << "State vec:\n";
        cout << "x:\t" << this->r.state_vec(0) << "\ty:\t" << this->r.state_vec(1) << "\ttheta:\t" << this->r.state_vec(2) << endl;
        for (int i = 3; i < this->r.state_vec.size(); i+=3) {
            cout << "r:\t" << this->r.state_vec(i) << "\tphi:\t" << this->r.state_vec(i+1) << "\ts:\t" << this->r.state_vec(i+2) << endl;
        }
    }

    void TR_test_moving() {
        cout << "Testing Robot movement...\n";
        // have a set of random directions and distances in which to move
        // for each one, try moving the robot in that direction
        int n_controls = 50;
        vector<Control> controls(n_controls, Control());
        bool flip = true;
        for (auto &control : controls) {
            control.v = 50.0;
            control.w = 20.0;
            if (flip) control.w *= -1;
            flip = !flip;
        }
        /*
        TODO: Add Robot::randomize() to randomize the position + x/y
        */
        // cout << "Using controls: " << endl;
        for (int i = 0; i < n_controls; i++) {
            // sample new pose
            controls[i].print();
            // cout << "Before move:\n";
            // this->r.print();
            Pose cur_pose = Pose(this->r.position, this->r.look_at);
            Pose new_pose = make_pose(this->r.sample_xt(controls[i], cur_pose));

            this->r.move_to_new_pose(new_pose, controls[i]);
            // cout << "After move:\n";
            // this->r.print();
        }
        // printj(this->r);
        cout << "Writing trajectory to .csv..." << endl;
        this->r.write_traj_to_csv();
        cout << "Trajectory written!" << endl;
        printj(this->r);
        cout << "Done with Robot tests\n";
    }

    void TR_test_sample_next_pose() {
        cout << "Sampling next poses...\n";
        // create init pose + control
        int n_poses = N_SAMPLES;
        double v = V;
        double w = W;
        Control ctrl(v, w);
        ctrl.print();
        Pose init_pose(this->r.position, this->r.look_at);
        init_pose.print();

        vector<Pose> poses(n_poses);
        for (int i = 0; i < n_poses; i++) {
            poses[i] = make_pose(this->r.sample_xt(ctrl, init_pose));
        }
        
        // write to csv
        std::ofstream out("sampled_poses.csv");
        // output header and init pose
        cout << "Writing to sampled_poses.csv...\n";
        out << "\"x\",\"y\",\"orientation\",\"v\",\"w\"\n";
        out << this->r.position.x() << ',' << this->r.position.y() << ',' << this->r.look_at << ',' << ctrl.v  << ',' << ctrl.w << "\n";
        for (const auto &pose : poses) {
            out << pose.position.x() << ',' << pose.position.y() << ',' << ',' << ",\n";
        }
        cout << "Output File written!" << endl;
    }

    void TR_test_sensing() {
        cout << "Testing sensing...\n";
        cout << "[test.cpp]Robot location: " << this->r.position.x() << ", " << this->r.position.y() << endl;
        MatrixXd landmarks = MatrixXd::Random(3, N_LANDMARKS);
        landmarks *= 25.0;
        cout << "Landmarks size: " << landmarks.rows() << ", " << landmarks.cols() << endl;
        MatrixXd features = this->r.sense_env(landmarks);
        print_features(features);
        to_csv(this->r.state_vec, "state_vector");
        print_state();
    }

    void TR_test_EKF_SLAM() {
        // init
        SPDLOG_INFO("Testing EKF SLAM...");
        cout << "[test.cpp]Robot location: " << this->r.position.x() << ", " << this->r.position.y() << endl;
        MatrixXd landmarks = MatrixXd::Random(3, N_LANDMARKS);
        landmarks *= 2.0;
        cout << "Landmarks size: " << landmarks.rows() << ", " << landmarks.cols() << endl;
        // run SLAM
        Vector2d control = {V, W};
        VectorXi c = VectorXi::LinSpaced(N_LANDMARKS, 1, N_LANDMARKS);
        VectorXd init_state_vec = VectorXd::Zero(N_LANDMARKS * 3 + 3);
        MatrixXd init_cov = MatrixXd::Zero(N_LANDMARKS * 3 + 3, N_LANDMARKS * 3 + 3);
        this->r.state_vec = init_state_vec; this->r.covariance = init_cov;
        for (int i = 0; i < N_STEPS; i++) {
            MatrixXd features = this->r.sense_env(landmarks);
            this->r.EKF_SLAM(this->r.state_vec, this->r.covariance, control, features, c);
            this->r.position = this->r.state_vec.head<2>();
            this->r.look_at = this->r.state_vec(2);
            string filename = fmt::format("state_vector_t{}", i);
            to_csv(this->r.state_vec, filename);
        }
    }

    void TR_test_Graph_SLAM() {
        SPDLOG_INFO("Testing Graph SLAM...\n");
        SPDLOG_INFO("Robot Location: {}, {}", this->r.position.x(), this->r.position.y());
        // init landmarks
        MatrixXd landmarks = MatrixXd::Random(3, N_LANDMARKS);
        landmarks *= 2.0;
        SPDLOG_INFO("Landmarks:\n{}", to_str(landmarks));

        // init correspondences
        VectorXi c = VectorXi::LinSpaced(N_LANDMARKS, 1, N_LANDMARKS);
        SPDLOG_INFO("Correspondences:\n{}", to_str(c));

        // init controls
        int n_controls = N_STEPS;
        VectorXd u = VectorXd::Zero(n_controls * 2);
        u(seq(0, last, 2)) = VectorXd::Constant(n_controls, V);
        u(seq(1, last, 2)) = VectorXd::Constant(n_controls, W);
        SPDLOG_INFO("Controls:\n{}", to_str(u));

        // init features
        vector<VectorXd> z;
        VectorXd z_t = this->r.sense_env(landmarks);
        z.push_back(z_t);
        SPDLOG_INFO("Features:\n{}", to_str(z_t));

        // init Graph SLAM
        VectorXd mu = this->r.Graph_SLAM_init(u);
        SPDLOG_INFO("mu:\n{}", to_str(mu));

        auto [omega, xi] = this->r.Graph_SLAM_linearize(u, z, c, mu);
        SPDLOG_INFO("Omega:\n{}\nXi:\n{}", to_str(omega), to_str(xi));
    }

    void run_tests() {
        // this->TR_test_moving();
        // this->TR_test_sample_next_pose();
        // this->TR_test_sensing();
        // this->TR_test_sensing();
        // this->TR_test_EKF_SLAM();
        this->TR_test_Graph_SLAM();
    }
};

struct TestWorld {
    World w;
    int n_landmarks = 50;
    TestWorld() {
        Point2d pos;
        pos.randomize(0.0, 1.0);
        // create random robot position
        Robot _r(pos.position.x(), pos.position.y());
        w.robot = _r;
        // add a bunch of random landmarks
        for (int i = 0; i < n_landmarks; i++) {
            Landmark lm;
            lm.randomize(-20.0, 20.0);
            w.landmarks.push_back(lm);
        }
    }

    void simple_test() {
        cout << "Testing world creation and printing of landmarks...\n";
        w.printlm();
        cout << "Creation/Print passed!";
    }

    void test_distance() {
        cout << "Testing world robot...\n";
        w.robot.print();
        for (const auto &l : w.landmarks) {
            cout << "Current LM:\n";
            l.print();
            double d = w.robot.distance_to(l);
            double correct = sqrt(pow(abs((double)l.x - (double)w.robot.position.x()), 2) + pow(abs((double)l.y - (double)w.robot.position.y()), 2));
            cout << "Distance: " << d << " expected: " << correct << endl;
            assert(isclose(d, correct));
            cout << "Distance " << w.robot.distance_to(l) << " is correct.\n";
        }
        cout << "Passed all distance tests." << endl;
    }

    void test_trajectory_lerp() {
        cout << "Testing LERP generation...\n";
        Vector2d end(10.0, 10.0);
        w.robot.generate_lerp_trajectory(Point2d(w.robot.position), Point2d(end), 100);
        printj(w.robot);
        cout << "Passed LERP Tests" << endl;
    }

    void run_tests() {
        // this->simple_test();
        // this->test_distance();
        // this->test_trajectory_lerp();
    }
};

int main() {
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%l] [%s:%#] %v");
    SPDLOG_DEBUG("######################## Running Tests ########################");

    TestRobot tr;
    tr.run_tests();

    SPDLOG_DEBUG("######################## Finished Tests ########################");
}