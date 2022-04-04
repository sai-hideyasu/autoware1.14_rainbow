cmake_minimum_required(VERSION 2.8.12)
project(arena_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_config_msgs
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
set(CMAKE_CXX_FLAGS "-O2 -Wall -Wno-unknown-pragmas ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arena_sdk
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

if(EXISTS "/etc/ld.so.conf.d/Arena_SDK.conf")
  set(ARENA_INC_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/include/Arena")
#  set(ARENAC_INC_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/include/ArenaC")
  set(ARENAGENTL_INC_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/include/GenTL")
  set(ARENASAVE_INC_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/include/Save")
#  set(ARENASAVEC_INC_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/include/SaveC")
  set(GENICAM_INC_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/GenICam/library/CPP/include")
  set(ARENASDK_LIB_DIR "$ENV{HOME}/ArenaSDK_Linux_x64/lib64")
  set(ARENASDK_LIBRARIES -larena -larenac -larenacd -larenad -lgentl -lgentld -lsave -lsavec -lsavecd -lsaved)
  set(FFMPEG_LIBRARIES -lavcodec -lavformat -lavutil -lswresample)
#  message(${ARENASDK_LIB_DIR})

  include_directories(
    ${ARENA_INC_DIR}
#    ${ARENAC_INC_DIR}
    ${ARENAGENTL_INC_DIR}
    ${ARENASAVE_INC_DIR}
#    ${ARENASAVEC_INC_DIR}
    ${GENICAM_INC_DIR}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  add_executable(lucid_camera
    nodes/lucid_camera.cpp
  )

#  link_directories(${ARENASDK_LIB_DIR})

  target_link_libraries(
    lucid_camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    /home/sit/ArenaSDK_Linux_x64/lib64/libarena.so
    /home/sit/ArenaSDK_Linux_x64/lib64/libarenad.so
    /home/sit/ArenaSDK_Linux_x64/lib64/libgentl.so
    /home/sit/ArenaSDK_Linux_x64/lib64/libgentld.so
    /home/sit/ArenaSDK_Linux_x64/lib64/libsave.so
    /home/sit/ArenaSDK_Linux_x64/lib64/libsaved.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/libGenApi_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/libLog_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/liblog4cpp_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/libMathParser_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/libNodeMapData_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/libXmlParser_gcc421_v3_0.so
    /home/sit/ArenaSDK_Linux_x64/ffmpeg/libavcodec.so
    /home/sit/ArenaSDK_Linux_x64/ffmpeg/libavformat.so
    /home/sit/ArenaSDK_Linux_x64/ffmpeg/libavformat.so
    /home/sit/ArenaSDK_Linux_x64/ffmpeg/libswresample.so
  )
  install(TARGETS lucid_camera
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(FILES scripts/lucid_camera.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
else()
  message("'Arena SDK' is not installed. 'lucid_camera' will not be built.")
endif()



## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
# ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/arena_sdk.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/arena_sdk_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arena_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
