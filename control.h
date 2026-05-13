#ifndef CONTROL_H
#define CONTROL_H

#include <Eigen/Dense>

struct Control {
    Control();
    Control(double trv, double orv);
    double tr_velocity;
    double or_velocity;
    void print() const;
};

#endif