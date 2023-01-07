cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_image_converter)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/pointcloud_to_image.cpp
)

############
# BINARIES #
############
cs_add_executable(pc2img_node
  src/pc2img_node.cpp
)
target_link_libraries(pc2img_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
