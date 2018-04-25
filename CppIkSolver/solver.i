%module solver

%{
#include "solver.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <stl.i>

%template(DoubleVector) std::vector<double>;

%include "solver.h"