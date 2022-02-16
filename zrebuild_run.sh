#!/bin/bash
CFG=${1:-Release}
./zbuild.sh ${CFG}
cd build_${CFG}/solver/
./solver
