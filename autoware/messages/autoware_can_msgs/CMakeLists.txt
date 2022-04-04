cmake_minimum_required(VERSION 2.8.3)
project(autoware_can_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    CANData.msg
    CANInfo.msg
    CANPacket.msg

    #add saiko_car_ware
    MicroBusCan501.msg
    MicroBusCan502.msg
    MicroBusCan503.msg
    MicroBusPseudoOperation.msg
    MicroBusPseudoParams.msg
    MicroBusCanSenderStatus.msg
    MicroBusCanVelocityParam.msg
    MicroBusSHHV.msg
    MicroBusSHHVArrange.msg
    MicroBusSPPM.msg
    SteerProofreading.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
