cmake_minimum_required(VERSION 2.8.3)
project(opencv_ros)
find_package(catkin REQUIRED roscpp std_msgs sensor_msgs cv_bridge image_transport)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
#  CATKIN_DEPENDS roscpp visualization_msgs robot_model_helper compute_cdist
#  DEPENDS assimp Eigen mlpack
)
add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})
include_directories(${catkin_INCLUDE_DIRS})
