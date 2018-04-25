cmake_minimum_required(VERSION 2.8.3)
project(ik_solver)

set(BASE_FILE_NAME solver)

set(CMAKE_BUILD_TYPE Release)

include_directories(/usr/include/eigen3)
include_directories(/usr/include/boost/)
include_directories(/opt/ros/kinetic/include/)
include_directories(/opt/ros/kinetic/include/kdl)

set(PYTHON_LIBRARY /usr/lib/x86_64-linux-gnu/libpython2.7.so)
set(PYTHON_INCLUDE_DIRS /usr/include/python2.7)

include_directories(${PYTHON_INCLUDE_DIRS})

#usage of link_directories is usually not recommended
link_directories(/opt/ros/kinetic/lib/)
link_directories(${PYTHON_LIBRARY})

#this is recommended
#find_library(OROCOS-KDL orocos-kdl /opt/ros/hydro/lib/)
#why am I not using this? I am too lazy to do it for all the libs we might need
#you should probably not follow my approach

#use swig OFF if you want to use swig.sh which might be more explanatory
option( USE_SWIG "Swig with CMAKE" OFF )

#set(CMAKE_CXX_FLAGS -Wl,defs)

set(MY_ADDED_LIBS orocos-kdl;kdl_parser;urdf;python2.7)

if(USE_SWIG)
  find_package(SWIG)
  if (SWIG_FOUND)
    SET (UseSWIG ON)
    INCLUDE(${SWIG_USE_FILE})  

    SET(CMAKE_SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    
    set(CMAKE_SWIG_OUTDIR "PyIkSolver")
    set(${SWIG_CXX_EXTENSION}, cpp)
    
    #SET (COMMON_LIBRARIES_PATH "${CMAKE_CURRENT_BINARY_DIR}/lib" CACHE PATH "Path where libraries common to all target languages are created.")
    
    SET_SOURCE_FILES_PROPERTIES(${BASE_FILE_NAME}.i PROPERTIES CPLUSPLUS ON)
    #SET_SOURCE_FILES_PROPERTIES(${BASE_FILE_NAME}.i PROPERTIES SWIG_FLAGS "-includeall")
      
    SWIG_ADD_MODULE(${BASE_FILE_NAME} java ${BASE_FILE_NAME}.i ${BASE_FILE_NAME}.cpp)
    SWIG_LINK_LIBRARIES(${BASE_FILE_NAME} ${JAVA_LIBRARIES} ${MY_ADDED_LIBS}) 
    
  else()
    message(STATUS "NOTICE: SWIG not found, JAVA bindings will not be created.")
  endif()
else()
  add_library(_solver SHARED  CppIkSolver/solver_wrap.cpp CppIkSolver/solver.cpp)
  target_link_libraries(_solver ${MY_ADDED_LIBS})
  SET_TARGET_PROPERTIES(_solver PROPERTIES PREFIX "")

  add_library(_solver_test CppIkSolver/solver_wrap.cpp CppIkSolver/solver.cpp)
  target_link_libraries(_solver_test ${MY_ADDED_LIBS})
  
  add_executable(solver_test CppIkSolver/solver_test.cpp)
  target_link_libraries(solver_test _solver)
endif()

#add_library(swig_kdl SHARED  base-kinematics_wrap.cpp base-kinematics.cpp)
#target_link_libraries(swig_kdl orocos-kdl kdl_parser)
