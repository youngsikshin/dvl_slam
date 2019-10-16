cmake_minimum_required(VERSION 2.6.0)

project(dedvo)

add_definitions(-std=c++11)
add_definitions("-DENABLE_SSE")

SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

find_package(OpenMP REQUIRED)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set (HEADER 
  System.h
  Datatypes.h
  CameraModel.h
  PinholeModel.h
  Config.h
  Point.h
  Frame.h
  Keyframe.h
  KeyframeWindow.h
  KeyframeDB.h
  Tracker.h
  Utility.h
  WindowOptimizer.h
  WindowOptimizer2.h
  GraphOptimizer.h
  ORBExtractor.h
  ORBMatcher.h
  ORBVocaburary.h
  LoopClosing.h
  Conversion.h
  g2o_type_se3sophus.h
  LocalTracker.h
  Tracker2.h
  WeightFunction.h
  LSQNonlinear.h
  LSQNonlinear.hpp
#  util/BotUtils.h
#  util/Datatypes.h
#  util/EigenSophusConversion.h
#  util/TimeStamp.h
)

set (SRC
  src/System.cpp
  src/PinholeModel.cpp
  src/Config.cpp
  src/Point.cpp
  src/Frame.cpp
  src/Keyframe.cpp
  src/KeyframeWindow.cpp
  src/KeyframeDB.cpp
  src/Tracker.cpp
  src/Utility.cpp
  src/WindowOptimizer.cpp
  src/WindowOptimizer2.cpp
  src/GraphOptimizer.cpp
  src/ORBExtractor.cpp
  src/ORBMatcher.cpp
  src/LoopClosing.cpp
  src/Conversion.cpp
  src/g2o_type_se3sophus.cpp
  src/LocalTracker.cpp
  src/Tracker2.cpp
  src/WeightFunction.cpp
#  src/LSQNonlinear.cpp
#  src/util/EigenSophusConversion.cpp
#  src/util/TimeStamp.cpp
)

## Find catkin and Packages
#find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(DLib REQUIRED)
find_package(DBoW2 REQUIRED)
find_library(G2O REQUIRED)
find_package(SuiteSparse REQUIRED)
find_library(cholmod REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CHOLMOD_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
)

set(IRPLIB_G2O ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_EIGEN} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SBA}) #)

add_library (dedvo ${SRC})
target_link_libraries (dedvo
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Sophus_LIBS}
   ${DBoW2_LIBS}
   ${DLib_LIBS}
#   ${PCL_LIBRARIES}
   ${CHOLMOD_LIBRARIES}
#   ${CSPARSE_LIBRARY}
   ${IRPLIB_G2O}
)

catkin_package(CATKIN_DEPENDS roscpp pcl_ros
               DEPENDS OpenCV Sophus
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})
