cmake_minimum_required(VERSION 2.8.3)
project(i2v)

#find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  autoware_config_msgs
  autoware_system_msgs
  roscpp
  std_msgs
  cryptography_aes
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/openssl
)

add_executable(i2v_tcp_sender
  src/sender_main.cpp
  src/i2v_tcp_sender.cpp
  src/sender/sender.cpp
  src/car_signal_info/car_signal_info.cpp
  src/walker_signal_info/walker_signal_info.cpp
  src/service_load_signal_info/service_load_signal_info.cpp
  src/reader/reader.cpp
  src/other.cpp
)
target_link_libraries(i2v_tcp_sender ${catkin_LIBRARIES} -lcrypto -lssl)
add_dependencies(i2v_tcp_sender ${catkin_EXPORTED_TARGETS})

add_executable(test
  src/test.cpp
  src/sender/sender.cpp
  src/car_signal_info/car_signal_info.cpp
  src/walker_signal_info/walker_signal_info.cpp
  src/service_load_signal_info/service_load_signal_info.cpp
  src/reader/reader.cpp
  src/other.cpp
)
target_link_libraries(test ${catkin_LIBRARIES} -lcrypto -lssl)
add_dependencies(test ${catkin_EXPORTED_TARGETS})

install(TARGETS i2v_tcp_sender test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
