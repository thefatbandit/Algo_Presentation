#!/bin/bash

cd ..
g++ -std=c++11 -c geometry_a_star.h a_star.cpp
g++ a_star.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
./sfml-app