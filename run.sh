#!/bin/bash

g++ -std=c++11 -c geometry.h heuristic.cpp
g++ heuristic.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
./sfml-app