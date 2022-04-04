cmake_minimum_required(VERSION 2.8.3)
project(cryptography_aes)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cryptography_aes
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/openssl
)

add_library(cryptography_aes src/aes.cpp)
add_dependencies(cryptography_aes ${catkin_EXPORTED_TARGETS})
target_link_libraries(cryptography_aes ${catkin_LIBRARIES} -lcrypto -lssl)


install(TARGETS cryptography_aes
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(DIRECTORY launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
