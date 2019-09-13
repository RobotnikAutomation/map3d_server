cmake_minimum_required(VERSION 2.8.3)
project(map3d_server)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map3d_server
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map3d_server src/map3d_server.cpp)
add_dependencies(map3d_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map3d_server
  ${catkin_LIBRARIES}
)

add_executable(map3d_saver src/map3d_saver.cpp)
add_dependencies(map3d_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map3d_saver
  ${catkin_LIBRARIES}
)

add_executable(map3d_to_costmap src/map3d_to_costmap.cpp)
add_dependencies(map3d_to_costmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map3d_to_costmap
  ${catkin_LIBRARIES}
)

install(TARGETS map3d_server map3d_saver map3d_to_costmap
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

