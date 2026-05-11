#include "landmark.h"
#include "point2d.h"
#include "robot.h"
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
            this->r.move_in_direction(moves[i]);
            // rotate in new direction by i
            this->r.look_at += i;
        }
        cout << "Printing Trajectory...\n";
        for (const auto &t : this->r.trajectory) {
            t.print();
        }
        cout << "Done with Robot tests";
    }

    void run_tests() {
        this->TR_test_moving();
    }
};

int main() {
    cout << "######################## Running Tests ########################\n";
    // LM_test_eigen_assignment();
    // LM_test_similarity();
    // LM_test_sorting();
    // P_test_eigen_assignment();
    // P_test_similarity();
    // P_test_sorting();
    TestRobot tr;
    tr.run_tests();
    cout << "######################## All Tests Passed ########################\n";
}