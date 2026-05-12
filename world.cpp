#include <vector>
#include <Eigen/Dense>
#include "world.h"
#include "landmark.h"
#include "robot.h"
using std::vector;
using std::cout;

World::World() : robot(Robot(0.0, 0.0)), landmarks({Landmark()}) {}
World::World(vector<Landmark> _landmarks) : robot(Robot(0.0, 0.0)), landmarks(_landmarks) {}
World::World(vector<Landmark> _landmarks, Robot _r) : robot(_r), landmarks(_landmarks) {}

void World::printlm() const {
    cout << "World Landmarks:\n";
    for (const auto &lm : this->landmarks) {
        lm.print();
    }
    cout << std::endl;
}