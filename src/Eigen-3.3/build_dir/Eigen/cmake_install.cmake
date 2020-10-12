# Install script for directory: /home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Cholesky"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/CholmodSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Core"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Dense"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Eigen"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Eigenvalues"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Geometry"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Householder"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/IterativeLinearSolvers"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Jacobi"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/LU"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/MetisSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/OrderingMethods"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/PaStiXSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/PardisoSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/QR"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/QtAlignedMalloc"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SPQRSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SVD"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Sparse"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseCholesky"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseCore"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseLU"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseQR"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/StdDeque"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/StdList"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/StdVector"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SuperLUSupport"
    "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/german/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

