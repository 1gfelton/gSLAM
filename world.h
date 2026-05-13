#ifndef WORLD_H
#define WORLD_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "point2d.h"
#include "landmark.h"
#include "robot.h"

struct World {
    World();
    World(std::vector<Landmark> _landmarks);
    World(std::vector<Landmark> _landmarks, Robot _r);
    Robot robot;
    std::vector<Landmark> landmarks;
    Eigen::Vector2d origin = {0.f, 0.f};
    void printlm() const;
};

#endif