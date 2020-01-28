# Install script for directory: /local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bfl/filter" TYPE FILE FILES
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/filter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/filter.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/particlefilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/particlefilter.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/bootstrapfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/bootstrapfilter.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/asirfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/asirfilter.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/kalmanfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/extendedkalmanfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/iteratedextendedkalmanfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/EKparticlefilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/SRiteratedextendedkalmanfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/innovationCheck.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/histogramfilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/histogramfilter.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/mixtureParticleFilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/mixtureParticleFilter.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/mixtureBootstrapFilter.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/filter/mixtureBootstrapFilter.cpp"
    )
endif()

