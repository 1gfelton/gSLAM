#!/bin/bash
g++ -I /usr/include/eigen3 control.cpp pose.cpp world.cpp utils.cpp test.cpp robot.cpp landmark.cpp point2d.cpp -o test
