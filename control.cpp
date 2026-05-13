#include "control.h"
#include <Eigen/Dense>
#include <iostream>
using std::cout;

Control::Control() : tr_velocity(0.0), or_velocity(0.0) {};
Control::Control(double trv, double orv) : tr_velocity(trv), or_velocity(orv) {};
void Control::print() const {
    cout << "Control: (" << this->tr_velocity << ", " << this->or_velocity << ")\n";
};