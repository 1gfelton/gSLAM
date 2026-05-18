#ifndef LANDMARK_H
#define LANDMARK_H
/*
*************************** Landmark Class ************************************
- Defines landmarks as points in the world that can be randomly generated
- For now the points are in 2D space
- maybe i shld do a more general point class from which Landmark would inherit?
    - bc right now trajectory would need to be a vector of landmarks which i don't want to do.....
    - or it could be a matrix of size N x 2? maybe this makes most sense to take advantage of Eigen...
*******************************************************************************
*/

#include <istream>
#include <Eigen/Dense>

struct Landmark {
    Landmark();
    // init with x and y values
    Landmark(double _x, double _y);
    // init with a 2-vector containing x and y
    Landmark(Eigen::Vector2d v);

    double x;
    double y;
    Eigen::Vector2d position;

    // Assign random values
    void randomize(const double &lower_bound, const double &upper_bound);
    void print() const;
};

bool operator<(const Landmark& a, const Landmark& b);
bool operator>(const Landmark& a, const Landmark& b);
bool operator<=(const Landmark& a, const Landmark& b);
bool operator>=(const Landmark& a, const Landmark& b);
bool operator==(const Landmark& a, const Landmark& b);

#endif