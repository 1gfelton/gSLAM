#ifndef CONTROL_H
#define CONTROL_H

#include <Eigen/Dense>

struct Control {
    Control();
    Control(double v, double w);
    double v;
    double w;
    void print() const;
};

#endif