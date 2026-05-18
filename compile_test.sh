#!/bin/bash
g++ -std=c++17 -I /usr/include/eigen3 -I . control.cpp pose.cpp world.cpp utils.cpp test.cpp robot.cpp landmark.cpp point2d.cpp -o test
