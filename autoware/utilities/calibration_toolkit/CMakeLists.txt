cmake_minimum_required(VERSION 2.8.3)
project(calibration_toolkit)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  sensor_msgs
  glviewer
  rosinterface
  cv_bridge
  pcl_ros
  image_transport
  autoware_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES calibration_toolkit
)

include_directories(
  CalibrationToolkit
  nodes/calibration_toolkit
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

## Declare a cpp library
add_library(calibrationtoolkit
        CalibrationToolkit/calibrationtoolkit.cpp
        CalibrationToolkit/calibrationtoolkit.h
        CalibrationToolkit/selectionwidget.cpp
        CalibrationToolkit/selectionwidget.h
        )
add_dependencies(calibrationtoolkit
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(calibrationtoolkit
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        nlopt
        )

## 2D
add_executable(calibration_test
        src/calibration_test/chessboard.cpp
        src/calibration_test/trans.cpp
        src/calibration_test/scan_window.cpp
        src/calibration_test/common_2d_calib.cpp
        src/calibration_test/image_window.cpp
        src/calibration_test/camera_lidar2d_offline_calib.cpp)

set(PARAM_YAML "${CMAKE_CURRENT_SOURCE_DIR}/param.yaml")
set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

set_target_properties(calibration_test
        PROPERTIES COMPILE_FLAGS
        "-DPARAM_YAML=${PARAM_YAML} -DCAMERA_YAML=${CAMERA_YAML}")

target_link_libraries(calibration_test
        ${catkin_LIBRARIES} ${OpenCV_LIBS} xml2 ${OPENGL_LIBRARIES})

## 3D
qt5_wrap_ui(calibration_toolkit_ui_mainwindow src/calibration_toolkit/mainwindow.ui)

add_executable(calibration_toolkit
        src/calibration_toolkit/main.cpp
        src/calibration_toolkit/mainwindow.cpp
        src/calibration_toolkit/mainwindow.h
        ${calibration_toolkit_ui_mainwindow}
        )

set_target_properties(calibration_toolkit
        PROPERTIES COMPILE_FLAGS "-fPIC"
        )

## Specify libraries to link a library or executable target against
target_link_libraries(calibration_toolkit
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        calibrationtoolkit
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        nlopt
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        )

install(TARGETS calibrationtoolkit calibration_toolkit
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
