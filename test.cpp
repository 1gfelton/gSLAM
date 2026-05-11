#include "landmark.h"
#include "robot.h"
#include <bits/stdc++.h>
#include <Eigen/Dense>
using namespace std;

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
        cout << "Coords: " << ll.coords << '\n';
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

int main() {
    cout << "######################## Running Tests ########################\n";
    LM_test_eigen_assignment();
    LM_test_similarity();
    LM_test_sorting();
    cout << "######################## All Tests Passed ########################\n";
}