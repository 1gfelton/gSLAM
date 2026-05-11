#ifndef POINT2D_H
#define POINT2D_H

/*
Atomic class for 2D points, mostly used to build trajectory so we don't have to use Landmark class as points (because that is silly)
*/

#include <iostream>
#include <Eigen/Dense>

struct Point2d {
    Point2d();
    Point2d(float _x, float _y);
    Point2d(Eigen::Vector2f _v);

    float x; float y;
    Eigen::Vector2f coords;

    void randomize(const float &lower_bound, const float &upper_bound);
    void print() const;
};

bool operator<(const Point2d& a, const Point2d& b);
bool operator>(const Point2d& a, const Point2d& b);
bool operator<=(const Point2d& a, const Point2d& b);
bool operator>=(const Point2d& a, const Point2d& b);
bool operator==(const Point2d& a, const Point2d& b);

#endif