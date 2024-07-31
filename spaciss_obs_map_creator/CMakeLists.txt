cmake_minimum_required(VERSION 3.0.2)
project(spaciss_obs_map_creator)

add_compile_options(-std=c++11 -O3 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs

  pedestrian_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spaciss_obs_map_creator
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/spaciss_obs_map_creator.cpp)
add_executable(${PROJECT_NAME}_node src/spaciss_obs_map_creator_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})