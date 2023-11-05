#!/bin/bash
echo "Configuring and building MOV-SLAM ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 
