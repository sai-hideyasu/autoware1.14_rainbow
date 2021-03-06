cmake_minimum_required(VERSION 2.8.3)
project(autoware_connector)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  autoware_config_msgs
  autoware_can_msgs
  autoware_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_executable(can_status_translator
  nodes/can_status_translator/can_status_translator_node.cpp
  nodes/can_status_translator/can_status_translator_core.cpp
)

target_link_libraries(can_status_translator
  ${catkin_LIBRARIES}
)

add_dependencies(can_status_translator
  ${catkin_EXPORTED_TARGETS}
)

add_executable(can_odometry
  nodes/can_odometry/can_odometry_node.cpp
  nodes/can_odometry/can_odometry_core.cpp
)

target_link_libraries(can_odometry
  ${catkin_LIBRARIES}
)

add_dependencies(can_odometry
  ${catkin_EXPORTED_TARGETS}
)

add_executable(localizer_switch nodes/localizer_switch/localizer_switch.cpp)
target_link_libraries(localizer_switch ${catkin_LIBRARIES})
add_dependencies(localizer_switch ${catkin_EXPORTED_TARGETS})

add_executable(estimate_to_baselink nodes/estimate_to_baselink/estimate_to_baselink.cpp)
target_link_libraries(estimate_to_baselink ${catkin_LIBRARIES})
add_dependencies(estimate_to_baselink ${catkin_EXPORTED_TARGETS})

add_executable(localizer_smooth_transition nodes/localizer_smooth_transition/localizer_smooth_transition.cpp)
target_link_libraries(localizer_smooth_transition ${catkin_LIBRARIES})
add_dependencies(localizer_smooth_transition ${catkin_EXPORTED_TARGETS})

install(TARGETS can_status_translator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS can_odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS localizer_switch
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS estimate_to_baselink
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS localizer_smooth_transition
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
