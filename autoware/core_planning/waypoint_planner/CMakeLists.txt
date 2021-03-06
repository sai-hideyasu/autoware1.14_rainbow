cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    astar_search
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    autoware_can_msgs
    derived_object_msgs
    lanelet2_extension
    libwaypoint_follower
    pcl_conversions
    pcl_ros
    roscpp
    roslint
    std_msgs
    tf
    vector_map
)

catkin_package()

# TODO add all codes to roslint
roslint_cpp(src/velocity_set_lanelet2/velocity_set_lanelet2.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  astar_avoid
  src/astar_avoid/astar_avoid.cpp
  src/astar_avoid/astar_avoid_node.cpp
)

target_link_libraries(
  astar_avoid
  ${catkin_LIBRARIES}
)

add_dependencies(
  astar_avoid
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  temporary_stopper
  src/temporary_stopper/temporary_stopper.cpp
)

target_link_libraries(
  temporary_stopper
  ${catkin_LIBRARIES}
)

add_dependencies(
  temporary_stopper
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  local_waypoint_adjustment
  src/local_waypoint_adjustment/local_waypoint_adjustment.cpp
)

target_link_libraries(
  local_waypoint_adjustment
  ${catkin_LIBRARIES}
)

add_dependencies(
  local_waypoint_adjustment
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  astar_avoid
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  stopper_distance
  src/stopper_distance/stopper_distance.cpp
)

target_link_libraries(
  stopper_distance
  ${catkin_LIBRARIES}
)

add_dependencies(
  stopper_distance
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  clothoid_avoidance
  src/clothoid_avoidance/clothoid_avoidance.cpp
)

target_link_libraries(
  clothoid_avoidance
  ${catkin_LIBRARIES}
)

add_dependencies(
  clothoid_avoidance
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  velocity_set
  src/velocity_set/velocity_set.cpp
  src/velocity_set/velocity_set_path.cpp
  src/velocity_set/velocity_set_info.cpp
  src/velocity_set/libvelocity_set.cpp
)

target_link_libraries(
  velocity_set
  ${catkin_LIBRARIES}
)

add_dependencies(
  velocity_set
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  velocity_set_lanelet2
  src/velocity_set_lanelet2/velocity_set_lanelet2.cpp
  src/velocity_set/velocity_set_path.cpp
  src/velocity_set/velocity_set_info.cpp
  src/velocity_set/libvelocity_set.cpp
)

target_link_libraries(
  velocity_set_lanelet2
  ${catkin_LIBRARIES}
)

add_dependencies(
  velocity_set_lanelet2
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  mobileye_transform
  src/mobileye_transform/mobileye_transform.cpp
)

target_link_libraries(
  mobileye_transform
  ${catkin_LIBRARIES}
)

add_dependencies(
  mobileye_transform
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  mobileye_tracker
  src/mobileye_tracker/mobileye_tracker.cpp
)

target_link_libraries(
  mobileye_tracker
  ${catkin_LIBRARIES}
)

add_dependencies(
  mobileye_tracker
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  esr_transform
  src/esr_transform/esr_transform.cpp
)

target_link_libraries(
  esr_transform
  ${catkin_LIBRARIES}
)

add_dependencies(
  esr_transform
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  esr_tracker
  src/esr_tracker/esr_tracker.cpp
)

target_link_libraries(
  esr_tracker
  ${catkin_LIBRARIES}
)

add_dependencies(
  esr_tracker
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS astar_avoid velocity_set velocity_set_lanelet2 temporary_stopper stopper_distance local_waypoint_adjustment
  clothoid_avoidance mobileye_transform mobileye_tracker esr_transform esr_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
