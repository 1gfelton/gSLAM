#ifndef POINT2D_H
#define POINT2D_H

/*
Atomic class for 2D points, mostly used to build trajectory so we don't have to use Landmark class as points (because that is silly)
*/

#include <iostream>
#include <Eigen/Dense>

struct Point2d {
    Point2d();
    Point2d(double _x, double _y);
    Point2d(Eigen::Vector2d _v);

    Eigen::Vector2d position;

    void randomize(const double &lower_bound, const double &upper_bound);
    void print() const;
    double distance_to(Point2d other);
};

bool operator<(const Point2d& a, const Point2d& b);
bool operator>(const Point2d& a, const Point2d& b);
bool operator<=(const Point2d& a, const Point2d& b);
bool operator>=(const Point2d& a, const Point2d& b);
bool operator==(const Point2d& a, const Point2d& b);

#endif