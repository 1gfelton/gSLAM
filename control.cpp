#include "control.h"
#include <Eigen/Dense>
#include <iostream>
using std::cout;

Control::Control() : v(0.0), w(0.0) {};
Control::Control(double trv, double orv) : v(trv), w(orv) {};
void Control::print() const {
    cout << "Control: (" << this->v << ", " << this->w << ")\n";
};