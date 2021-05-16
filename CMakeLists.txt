cmake_minimum_required(VERSION 2.8.3)
project(cloud_segmentation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/cloud_segmentation.cc
)

cs_add_executable(cloud_segmentation_node
  src/node.cc
)
target_link_libraries(cloud_segmentation_node ${PROJECT_NAME})

cs_install()
cs_export()
