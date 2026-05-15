#include "utils.h"
#include <math.h>
using std::pair;
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

/* from Probabilistic Robotics by Thrun et al., 2006 */
double triangle_distribution(double a, double sigma) {
    return std::max(0.0, (1 / (sqrt(6) * sqrt(sigma))) - (abs(a) / (6 * sigma)));
}