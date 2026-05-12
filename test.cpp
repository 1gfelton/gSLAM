#include "world.h"
#include "landmark.h"
#include "point2d.h"
#include "robot.h"
#include "utils.h"
#include <bits/stdc++.h>
#include <Eigen/Dense>
using namespace std;

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
        t.print();
    }
    cout << endl;
}


struct TestRobot {
    Robot r;
    TestRobot() {
        Point2d random_pos;
        random_pos.randomize(0.0, 1.0);
        // create random robot looking at an angle 45 deg from x-axis
        Robot _r(random_pos.x, random_pos.y, 45.0);
        r = _r;
    }

    void TR_test_moving() {
        cout << "Testing Robot movement...\n";
        // have a set of random directions and distances in which to move
        // for each one, try moving the robot in that direction
        int n_moves = 10;
        vector<float> moves(n_moves, 0.0);
        for (int i = 0; i < n_moves; i++) {
            moves[i] += i;
        }
        for (int i = 0; i < n_moves; i++) {
            // move by i
            cout << "Moving by magnitude " << moves[i] << endl;
            this->r.move_in_direction(moves[i]);
            // rotate in new direction by i
            cout << "Rotating by " << this->r.look_at + i << endl;
            this->r.look_at += i;
            this->r.print();
        }
        printj(this->r);
        cout << "Done with Robot tests\n";
    }
    void run_tests() {
        this->TR_test_moving();
    }
};

struct TestWorld {
    World w;
    int n_landmarks = 50;
    TestWorld() {
        Point2d random_pos;
        random_pos.randomize(0.0, 1.0);
        // create random robot looking at an angle 45 deg from x-axis
        Robot _r(random_pos.x, random_pos.y, 45.0);
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
            float d = w.robot.distance_to(l);
            float correct = sqrt(pow(abs((double)l.x - (double)w.robot.x), 2) + pow(abs((double)l.y - (double)w.robot.y), 2));
            cout << "Distance: " << d << " expected: " << correct << endl;
            assert(isclose(d, correct));
            cout << "Distance " << w.robot.distance_to(l) << " is correct.\n";
        }
        cout << "Passed all distance tests." << endl;
    }

    void test_trajectory_lerp() {
        cout << "Testing LERP generation...\n";
        Eigen::Vector2f end(10.0, 10.0);
        w.robot.generate_lerp_trajectory(w.robot.position, Point2d(end), 100);
        printj(w.robot);
        cout << "Passed LERP Tests" << endl;
    }

    void run_tests() {
        // this->simple_test();
        this->test_distance();
        this->test_trajectory_lerp();
    }
};

int main() {
    cout << "######################## Running Tests ########################\n";
    // TestRobot tr;
    // tr.run_tests();

    TestWorld tw;
    tw.run_tests();
    cout << "######################## All Tests Passed ########################\n";
}