#!/bin/bash

cd ..
g++ -std=c++11 -c geometry.h dijkstra.cpp
g++ dijkstra.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
./sfml-app