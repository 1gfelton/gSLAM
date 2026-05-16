#include "utils.h"
#include <math.h>
#include <random>
#include <array>
using std::pair;
using std::array;
/*
from: https://stackoverflow.com/questions/31502120/sin-and-cos-give-unexpected-results-for-well-known-angles/31525208#31525208
*/
double to_radians(float deg) { 
    return ((double) deg / 180.0) * (double) M_PI;
}

double sind(double x) {
    if (!isfinite(x)) {
        return sin(x);
    }
    if (x < 0.0) {
        return -sind(-x);
    }
    int quo;
    double x90 = remquo(fabs(x), 90.0, &quo);
    switch(quo % 4) {
        case 0:
            return sin(to_radians(x90) * 1.0);
        case 1:
            return cos(to_radians(x90));
        case 2:
            return sin(to_radians(-x90) * 1.0);
        case 3:
            return -cos(to_radians(x90));
    }
    return 0.0;
}

double cosd(double x) {
    if (!isfinite(x)) {
        return cos(x);
    }
    if (x < 0.0) {
        return -cosd(-x);
    }
    int quo;
    double x90 = remquo(fabs(x), 90.0, &quo);
    switch(quo % 4) {
        case 0:
            return cos(to_radians(x90) * 1.0);
        case 1:
            return sin(to_radians(x90));
        case 2:
            return cos(to_radians(-x90) * 1.0);
        case 3:
            return -sin(to_radians(x90));
    }
    return 0.0;
}

/*
from `np.isclose()`
*/
bool isclose(double a, double b, double rtol, double atol) {
    return std::abs(a - b) <= atol + rtol * std::abs(b);
}

/* Convenience method to return a new `pair<Pose, Control>` when making the trajectory */
pair<Pose, Control> make_traj_position(Eigen::Vector2d pos, double theta, double v, double w) {
    return std::make_pair(Pose(pos, theta), Control(v, w));
}

/* from https://stackoverflow.com/questions/55681324/is-there-a-function-to-generate-random-number-using-triangular-distribution-in-c */
std::piecewise_linear_distribution<double> triangular_distribution(double lower, double mu, double upper) {
    array<double, 3> i{lower, mu, upper};
    array<double, 3> w{0, 1, 0};
    return std::piecewise_linear_distribution<double>{i.begin(), i.end(), w.begin()};
}

double sample_triangular_dist(double lower, double mu, double upper) {
    std::mt19937 gen(std::random_device);
    auto dist = triangular_distribution(lower, mu, upper);
    return dist(gen);
}