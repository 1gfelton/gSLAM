#!/bin/bash
g++ -I /usr/include/eigen3 world.cpp utils.cpp test.cpp robot.cpp landmark.cpp point2d.cpp -o test
