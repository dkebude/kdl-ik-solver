# This file was automatically generated by SWIG (http://www.swig.org).
# Version 2.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2,6,0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_solver', [dirname(__file__)])
        except ImportError:
            import _solver
            return _solver
        if fp is not None:
            try:
                _mod = imp.load_module('_solver', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _solver = swig_import_helper()
    del swig_import_helper
else:
    import _solver
del version_info
try:
    _swig_property = property
except NameError:
    pass # Python < 2.2 doesn't have 'property'.
def _swig_setattr_nondynamic(self,class_type,name,value,static=1):
    if (name == "thisown"): return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name,None)
    if method: return method(self,value)
    if (not static):
        self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)

def _swig_setattr(self,class_type,name,value):
    return _swig_setattr_nondynamic(self,class_type,name,value,0)

def _swig_getattr(self,class_type,name):
    if (name == "thisown"): return self.this.own()
    method = class_type.__swig_getmethods__.get(name,None)
    if method: return method(self)
    raise AttributeError(name)

def _swig_repr(self):
    try: strthis = "proxy of " + self.this.__repr__()
    except: strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object : pass
    _newclass = 0


class SwigPyIterator(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SwigPyIterator, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SwigPyIterator, name)
    def __init__(self, *args, **kwargs): raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _solver.delete_SwigPyIterator
    __del__ = lambda self : None;
    def value(self): return _solver.SwigPyIterator_value(self)
    def incr(self, n=1): return _solver.SwigPyIterator_incr(self, n)
    def decr(self, n=1): return _solver.SwigPyIterator_decr(self, n)
    def distance(self, *args): return _solver.SwigPyIterator_distance(self, *args)
    def equal(self, *args): return _solver.SwigPyIterator_equal(self, *args)
    def copy(self): return _solver.SwigPyIterator_copy(self)
    def next(self): return _solver.SwigPyIterator_next(self)
    def __next__(self): return _solver.SwigPyIterator___next__(self)
    def previous(self): return _solver.SwigPyIterator_previous(self)
    def advance(self, *args): return _solver.SwigPyIterator_advance(self, *args)
    def __eq__(self, *args): return _solver.SwigPyIterator___eq__(self, *args)
    def __ne__(self, *args): return _solver.SwigPyIterator___ne__(self, *args)
    def __iadd__(self, *args): return _solver.SwigPyIterator___iadd__(self, *args)
    def __isub__(self, *args): return _solver.SwigPyIterator___isub__(self, *args)
    def __add__(self, *args): return _solver.SwigPyIterator___add__(self, *args)
    def __sub__(self, *args): return _solver.SwigPyIterator___sub__(self, *args)
    def __iter__(self): return self
SwigPyIterator_swigregister = _solver.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)

class DoubleVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, DoubleVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, DoubleVector, name)
    __repr__ = _swig_repr
    def iterator(self): return _solver.DoubleVector_iterator(self)
    def __iter__(self): return self.iterator()
    def __nonzero__(self): return _solver.DoubleVector___nonzero__(self)
    def __bool__(self): return _solver.DoubleVector___bool__(self)
    def __len__(self): return _solver.DoubleVector___len__(self)
    def pop(self): return _solver.DoubleVector_pop(self)
    def __getslice__(self, *args): return _solver.DoubleVector___getslice__(self, *args)
    def __setslice__(self, *args): return _solver.DoubleVector___setslice__(self, *args)
    def __delslice__(self, *args): return _solver.DoubleVector___delslice__(self, *args)
    def __delitem__(self, *args): return _solver.DoubleVector___delitem__(self, *args)
    def __getitem__(self, *args): return _solver.DoubleVector___getitem__(self, *args)
    def __setitem__(self, *args): return _solver.DoubleVector___setitem__(self, *args)
    def append(self, *args): return _solver.DoubleVector_append(self, *args)
    def empty(self): return _solver.DoubleVector_empty(self)
    def size(self): return _solver.DoubleVector_size(self)
    def clear(self): return _solver.DoubleVector_clear(self)
    def swap(self, *args): return _solver.DoubleVector_swap(self, *args)
    def get_allocator(self): return _solver.DoubleVector_get_allocator(self)
    def begin(self): return _solver.DoubleVector_begin(self)
    def end(self): return _solver.DoubleVector_end(self)
    def rbegin(self): return _solver.DoubleVector_rbegin(self)
    def rend(self): return _solver.DoubleVector_rend(self)
    def pop_back(self): return _solver.DoubleVector_pop_back(self)
    def erase(self, *args): return _solver.DoubleVector_erase(self, *args)
    def __init__(self, *args): 
        this = _solver.new_DoubleVector(*args)
        try: self.this.append(this)
        except: self.this = this
    def push_back(self, *args): return _solver.DoubleVector_push_back(self, *args)
    def front(self): return _solver.DoubleVector_front(self)
    def back(self): return _solver.DoubleVector_back(self)
    def assign(self, *args): return _solver.DoubleVector_assign(self, *args)
    def resize(self, *args): return _solver.DoubleVector_resize(self, *args)
    def insert(self, *args): return _solver.DoubleVector_insert(self, *args)
    def reserve(self, *args): return _solver.DoubleVector_reserve(self, *args)
    def capacity(self): return _solver.DoubleVector_capacity(self)
    __swig_destroy__ = _solver.delete_DoubleVector
    __del__ = lambda self : None;
DoubleVector_swigregister = _solver.DoubleVector_swigregister
DoubleVector_swigregister(DoubleVector)

PI = _solver.PI

def fromFile(*args):
  return _solver.fromFile(*args)
fromFile = _solver.fromFile

def doubleVecToJntArray(*args):
  return _solver.doubleVecToJntArray(*args)
doubleVecToJntArray = _solver.doubleVecToJntArray

def doubleVecToFrame(*args):
  return _solver.doubleVecToFrame(*args)
doubleVecToFrame = _solver.doubleVecToFrame

def jntArrayToDoubleVec(*args):
  return _solver.jntArrayToDoubleVec(*args)
jntArrayToDoubleVec = _solver.jntArrayToDoubleVec

def frameToDoubleVec(*args):
  return _solver.frameToDoubleVec(*args)
frameToDoubleVec = _solver.frameToDoubleVec

def printDoubleVec(*args):
  return _solver.printDoubleVec(*args)
printDoubleVec = _solver.printDoubleVec

def printJntArray(*args):
  return _solver.printJntArray(*args)
printJntArray = _solver.printJntArray

def e_dist(*args):
  return _solver.e_dist(*args)
e_dist = _solver.e_dist

def vecNormalRandom(*args):
  return _solver.vecNormalRandom(*args)
vecNormalRandom = _solver.vecNormalRandom

def normalizeQuaternion(*args):
  return _solver.normalizeQuaternion(*args)
normalizeQuaternion = _solver.normalizeQuaternion
class kdlSolver(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, kdlSolver, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, kdlSolver, name)
    __repr__ = _swig_repr
    __swig_destroy__ = _solver.delete_kdlSolver
    __del__ = lambda self : None;
    def __init__(self, *args): 
        this = _solver.new_kdlSolver(*args)
        try: self.this.append(this)
        except: self.this = this
    def getNumJoints(self): return _solver.kdlSolver_getNumJoints(self)
    def getJointLimits(self, *args): return _solver.kdlSolver_getJointLimits(self, *args)
    def randInit(self, *args): return _solver.kdlSolver_randInit(self, *args)
    def solveFkDouble(self, *args): return _solver.kdlSolver_solveFkDouble(self, *args)
    def solveFkJntArray(self, *args): return _solver.kdlSolver_solveFkJntArray(self, *args)
    def solvePoseIk(self, *args): return _solver.kdlSolver_solvePoseIk(self, *args)
    def solvePosOnlyIk(self, *args): return _solver.kdlSolver_solvePosOnlyIk(self, *args)
    def solveHybridIk(self, *args): return _solver.kdlSolver_solveHybridIk(self, *args)
    def clamp(self, *args): return _solver.kdlSolver_clamp(self, *args)
    def myCartToJntVel(self, *args): return _solver.kdlSolver_myCartToJntVel(self, *args)
    def reject(self, *args): return _solver.kdlSolver_reject(self, *args)
kdlSolver_swigregister = _solver.kdlSolver_swigregister
kdlSolver_swigregister(kdlSolver)

# This file is compatible with both classic and new-style classes.


