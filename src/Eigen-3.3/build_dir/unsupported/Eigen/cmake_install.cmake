# Install script for directory: /home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/AdolcForward"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/AlignedVector3"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/ArpackSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/AutoDiff"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/BVH"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/EulerAngles"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/FFT"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/IterativeSolvers"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/KroneckerProduct"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/LevenbergMarquardt"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/MatrixFunctions"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/MoreVectorization"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/MPRealSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/NonLinearOptimization"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/NumericalDiff"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/OpenGLSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/Polynomials"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/Skyline"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/SparseExtra"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/SpecialFunctions"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/build_dir/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

