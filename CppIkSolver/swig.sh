#! /bin/bash
PYTHON_DIR=../PyIkSolver
mkdir -p ../build
mkdir -p ${PYTHON_DIR}
swig -Wall -c++ -python -outdir ${PYTHON_DIR} -o solver_wrap.cpp solver.i 
cd ../build
cmake ..
make
mv _solver.so ../PyIkSolver 