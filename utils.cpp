#include "utils.h"
#include <math.h>
#include <random>
#include <algorithm>
#include <array>
#include <string>
#include <sstream>

using std::pair;
using std::array;
using namespace Eigen;
/*
from: https://stackoverflow.com/questions/31502120/sin-and-cos-give-unexpected-results-for-well-known-angles/31525208#31525208
*/
double to_radians(double deg) { 
    return ((double) deg / 180.0) * (double) M_PI;
}

/*
from `np.isclose()`
*/
bool isclose(double a, double b, double rtol, double atol) {
    return std::abs(a - b) <= atol + rtol * std::abs(b);
}

/* Convenience method to return a new `pair<Pose, Control>` when making the trajectory */
pair<Pose, Control> make_traj_position(Vector2d pos, double theta, double v, double w) {
    Vector2d p = pos(seq(0, 1));
    return std::make_pair(Pose(p, theta), Control(v, w));
}

double sample_triangular_dist(double mu, double sigma2) {
    std::random_device r;
    std::mt19937 gen{r()};
    auto dist = std::uniform_real_distribution<double>(-sqrt(sigma2), sqrt(sigma2));
    double a1 = dist(gen);
    double a2 = dist(gen);
    return (sqrt(6)/2) * (a1 + a2);
}

double triangular_prob(double mu, double sigma2) {
    return std::max(0.0, (1 / (sqrt(6) * sqrt(sigma2))) - (abs(mu)/(6 * sigma2)));
}

Pose make_pose(Vector3d v) {
    Vector2d vv = {v[0], v[1]};
    return Pose(vv, v[2]);
}

VectorXd to_cartesian(VectorXd z) {
    VectorXd ans = VectorXd::Zero(z.size());
    for (int i = 0; i < z.size(); i+= 3) {
        double r = z(i);
        double phi = z(i + 1);
        double s = z(i + 2);

        double x = r * cos(phi);
        double y = r * sin(phi);
        ans(seq(i, i + 2)) = Vector3d{x, y, s};
    }
    return ans;
}