cmake_minimum_required(VERSION 2.6.0)

project(dedvo_gui)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
add_definitions(-std=c++11)

set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(CMAKE_AUTOMOC ON)

## Find catkin and Packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    sensor_msgs
    image_transport
    cv_bridge
    pcl_ros
    pcl_conversions
    pcl_msgs
    dedvo
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

#find_package(Eigen3 REQUIRED)
#find_package(Sophus REQUIRED)

find_package (Qt5Widgets)
find_package (Qt5Gui)
find_package (Qt5OpenGL)
find_package (OpenCV REQUIRED)

find_package(G2O REQUIRED)
#find_package(SuiteSparse REQUIRED)
find_library(cholmod REQUIRED)
set(IRPLIB_G2O ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_EIGEN} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SBA}) #)

#if(SSSE4_FOUND)
#    add_definitions("-msse4")
#endif(SSSE4_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dedvo_gui OpenGL
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
    cv_bridge
    pcl_ros
    pcl_conversions
    pcl_msgs
    dedvo
)

qt5_wrap_ui (dedvo_gui_moc ${SRC_DIR}/MainWindow.ui)
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${SRC_DIR}
    ${CHOLMOD_INCLUDE_DIR}
)


set (HEADER
    include/dedvo_gui/MainWindow.h
    include/dedvo_gui/Resources.h
    include/dedvo_gui/RosClient.h
    include/dedvo_gui/CamLidarProcess.h
    include/dedvo_gui/DedvoGLWidget.h
)


add_executable (dedvo_gui 
    ${HEADER}
    src/main.cpp
    src/MainWindow.cpp
    src/RosClient.cpp
    src/CamLidarProcess.cpp
    src/DedvoGLWidget.cpp
    ${SHADER_RSC_ADDED}
    ${dedvo_gui_moc}
)

target_link_libraries (dedvo_gui
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
#    ${OPENGL_LIBRARIES}
    GL
    ${CHOLMOD_LIBRARIES}
    ${CSPARSE_LIBRARY}
#    ${IRPLIB_G2O}
)

install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
