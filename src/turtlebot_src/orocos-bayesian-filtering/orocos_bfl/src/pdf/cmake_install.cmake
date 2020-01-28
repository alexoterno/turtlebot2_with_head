# Install script for directory: /local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bfl/pdf" TYPE FILE FILES
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/pdf.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/conditionalpdf.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/discretepdf.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/discreteconditionalpdf.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/mcpdf.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/mcpdf.cpp"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/gaussian.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/uniform.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/conditionalgaussian.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/conditionalgaussian_additivenoise.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/analyticconditionalgaussian.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/analyticconditionalgaussian_additivenoise.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/linearanalyticconditionalgaussian.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/filterproposaldensity.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/EKF_proposaldensity.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/mixture.h"
    "/local_scratch/aauterna/ros_projects/turtlebot2/src/orocos-bayesian-filtering/orocos_bfl/src/pdf/mixture.cpp"
    )
endif()

