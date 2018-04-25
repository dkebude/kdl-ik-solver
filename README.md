# kdl-ik-solver

This is a KDL[http://www.orocos.org/kdl] based inverse kinematics solver written in C++ and converted to Python using swig.

If you would like to make any changes to the solver, do so in the files inside CppIkSolver folder.

To build and create a lib out of the files, you can either use cmake directly or use the swig.sh inside the CppIkSolver. Using swig.sh will also create the Python files inside PyIkSolver.