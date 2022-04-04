cmake_minimum_required(VERSION 2.8.3)
project(vehicle_socket)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_can_msgs
  autoware_config_msgs
  autoware_system_msgs
  autoware_msgs
  roscpp
  std_msgs
  tablet_socket_msgs
  geometry_msgs
  nmea_msgs
  roslib
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_receiver nodes/vehicle_receiver/vehicle_receiver.cpp)
target_link_libraries(vehicle_receiver ${catkin_LIBRARIES})
add_dependencies(vehicle_receiver ${catkin_EXPORTED_TARGETS})

add_executable(vehicle_sender nodes/vehicle_sender/vehicle_sender.cpp)
target_link_libraries(vehicle_sender ${catkin_LIBRARIES})
add_dependencies(vehicle_sender ${catkin_EXPORTED_TARGETS})

add_executable(kvaser_microbus_can_receiver
	nodes/kvaser_microbus_can_receiver/kvaser_microbus_can_receiver.cpp)
target_link_libraries(kvaser_microbus_can_receiver
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_receiver
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_microbus_can_sender
	nodes/kvaser_microbus_can_sender/kvaser_microbus_can_sender.cpp)
target_link_libraries(kvaser_microbus_can_sender
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_sender
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(rs232c_microbus_can_receiver
	nodes/rs232c_microbus_can_receiver/rs232c_microbus_can_receiver.cpp)
target_link_libraries(rs232c_microbus_can_receiver
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(rs232c_microbus_can_receiver
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(microbus_pseudo_can_publisher
	nodes/microbus_pseudo_can_publisher/microbus_pseudo_can_publisher.cpp)
target_link_libraries(microbus_pseudo_can_publisher
  ${catkin_LIBRARIES}
  )
add_dependencies(microbus_pseudo_can_publisher
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(microbus_pseudo_can_sender
	nodes/microbus_pseudo_can_sender/microbus_pseudo_can_sender.cpp)
target_link_libraries(microbus_pseudo_can_sender
  ${catkin_LIBRARIES}
  )
add_dependencies(microbus_pseudo_can_sender
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(log_split
	nodes/log_split/log_split.cpp)
target_link_libraries(log_split
  ${catkin_LIBRARIES}
  )
add_dependencies(log_split
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(steer_proofreading
	nodes/steer_proofreading/steer_proofreading.cpp)
target_link_libraries(steer_proofreading
  ${catkin_LIBRARIES}
  )
add_dependencies(steer_proofreading
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(rs232_proofreading
	nodes/steer_proofreading/rs232_proofreading.cpp)
target_link_libraries(rs232_proofreading
  ${catkin_LIBRARIES}
  )
add_dependencies(rs232_proofreading
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS vehicle_sender vehicle_receiver kvaser_microbus_can_sender kvaser_microbus_can_receiver
                rs232c_microbus_can_receiver microbus_pseudo_can_publisher microbus_pseudo_can_sender log_split
                steer_proofreading rs232_proofreading
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
