#!/bin/bash
CFG=${1:-Release}
echo "Build configuration: ${CFG}"
mkdir -p build_${CFG}
cd build_${CFG}
cmake -DCMAKE_BUILD_TYPE=${CFG} ../
make
