cmake_minimum_required(VERSION 2.8.3)
project(rviz_restart)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(rviz_restart src/rviz_restart.cpp)
target_link_libraries(rviz_restart ${catkin_LIBRARIES})
add_dependencies(rviz_restart
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS rviz_restart
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
