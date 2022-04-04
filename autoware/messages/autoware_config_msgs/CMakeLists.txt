cmake_minimum_required(VERSION 2.8.3)
project(autoware_config_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    ConfigApproximateNDTMapping.msg
    ConfigCarDPM.msg
    ConfigCarFusion.msg
    ConfigCarKF.msg
    ConfigCompareMapFilter.msg
    ConfigDecisionMaker.msg
    ConfigDistanceFilter.msg
    ConfigICP.msg
    ConfigLaneRule.msg
    ConfigLaneSelect.msg
    ConfigLaneStop.msg
    ConfigLatticeVelocitySet.msg
    ConfigNDT.msg
    ConfigNDTMapping.msg
    ConfigNDTMappingOutput.msg
    ConfigPedestrianDPM.msg
    ConfigPedestrianFusion.msg
    ConfigPedestrianKF.msg
    ConfigPlannerSelector.msg
    ConfigPoints2Polygon.msg
    ConfigRandomFilter.msg
    ConfigRayGroundFilter.msg
    ConfigRcnn.msg
    ConfigRingFilter.msg
    ConfigRingGroundFilter.msg
    ConfigSSD.msg
    ConfigTwistFilter.msg
    ConfigVelocitySet.msg
    ConfigVoxelGridFilter.msg
    ConfigWaypointFollower.msg
    ConfigWaypointReplanner.msg

    #saiko_car_ware add
    ConfigGnssLocalizer.msg
    ConfigCanOdometry.msg
    ConfigLocalizerSwitch.msg
    ConfigMicrobusPseudoCanPublisher.msg
    ConfigMicroBusCan.msg
    ConfigTemporaryStopper.msg
    ConfigMobileyeTracker.msg
    ConfigCurrentVelocityConversion.msg
    ConfigEstimateToBaselink.msg
    ConfigLookAheadRatioMagn.msg
    ConfigMileagePublisher.msg
    ConfigWaypointFileMove.msg
    ConfigCmdSelector.msg
    ConfigStraightLineFix.msg
    ConfigPeriodSignal.msg
    ConfigRegionTlr.msg
    ConfigWaypointAdjustment.msg
    ConfigTravelRouteRegistration.msg
    ConfigClothoidAvoidance.msg
    ConfigWaypointExtension.msg
    ConfigArenaSDK.msg
    ConfigLocalizerSmoothTransition.msg
    ConfigI2V.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
