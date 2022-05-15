#!/bin/bash

AUTOWARE_PATH="${HOME}/saiko_car_ware_2022_01_06_rainbow"
VELODYNE_APTH="${HOME}/velodyne_vls"
WAYPOINT_FILE="${HOME}/home/sit/load_data/sibusawa/rainbow/2-kitakouen-odaka/2021_03_19_end_fix.csv"

source /opt/ros/melodic/setup.bash
source ${VELODYNE_APTH}/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ${AUTOWARE_PATH}/install/local_setup.bash

sleep 2
echo "currency : source"

# roscore start
roscore &
sleep 3
echo "currency : roscore"

# Set /config/arena_sdk
rostopic pub /config/arena_sdk autoware_config_msgs/ConfigArenaSDK "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
degree: 0
gain: 3.0
save: false" -l &
sleep 1
echo "currency : /config/arena_sdk"

# Set arena_sdk
roslaunch arena_sdk lucid_camera.launch &
sleep 3
echo "currency : arena sdk"

#Set mobileye
roslaunch runtime_manager mobileye.launch &
sleep 3
echo "currency : mobileye"

# Load setup_tf.launch
#roslaunch runtime_manager setup_tab_rainbow.launch x:=6.2 y:=0.0 z:=2.5 yaw:=-0.04 pitch:=0.0 roll:=0.0 frame_id:=/base_link child_frame_id:=/velodyne period_in_ms:=10 &
roslaunch runtime_manager setup_tab_rainbow.launch &
sleep 3
echo "currency : setup_tf"

# Load map to world launch
roslaunch runtime_manager map_to_world.launch &
sleep 3
echo "currency : map to world launch"

roslaunch map_file points_map_loader.launch scene_num:=noupdate path_area_list:=None path_pcd:=/home/sit/load_data/okabe/pcd/old/0.50_Lzi1_rot.pcd,/home/sit/load_data/okabe/pcd/old/0.50_Lzi2_rot.pcd,/home/sit/load_data/okabe/pcd/old/0.50_tyokusen1_rot.pcd,/home/sit/load_data/okabe/pcd/old/0.50_tyokusen2_rot.pcd &
sleep 3
echo "currency : points_map_loade"

#Set vector_map_loader
#rosrun map_file vector_map_loader /home/sit/load_data/okabe/ADASmap3/ADASmap/area.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/crosswalk.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/curb.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/dtlane.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/idx.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/lane.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/line.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/node.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/point.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/pole.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/poledata.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/roadedge.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/signaldata.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/stopline.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/vector.csv /home/sit/load_data/okabe/ADASmap3/ADASmap/whiteline.csv &
sleep 2
echo "currency : vector_map_loade"

# Set novatel gnss 2
#roslaunch novatel_oem7 novatel_oem7_tcp.launch __ns:=novatel_oem7_2 ip:=192.168.1.151 &
roslaunch novatel_oem7 novatel_oem7.launch __ns:=novatel_oem7_2 port:=/dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_DMMU18180091Y-if00-port0 baud:=115200 &
sleep 3
echo "currency : gnss2"

#Set i2v
#roslaunch i2v i2v_tcp_sender_jousi.launch &
#sleep 3
#echo "currency : i2v"

# Set lidar
roslaunch runtime_manager VLS128_points_single.launch calibration:=${VELODYNE_APTH}/src/velodyne_pointcloud/params/VeloView_VLS-128_FS1.yaml &
sleep 3
echo "currency : velodyne 128"

# Set plane of gnss
roslaunch gnss_localizer nmea2tfpose_RTK2.launch plane:=9 nmea_topic:=/novatel_oem7_2/nmea_sentence name_space:=True &
sleep 3
echo "currency : nmea2tfpose"

#Set /config/gnss_localizer
rostopic pub __name:=config_gnss_localizer /config/gnss_localizer autoware_config_msgs/ConfigGnssLocalizer "header:
  seq: 1
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
use_topic_num: '2'
yaw_correct_deg: 0.0
position_y_correct: 0.0" -l &
sleep 1
echo "currency : /config/gnss_localizer"

#Set gnss_selector and gnss_localizer
roslaunch gnss_localizer gnss_matching.launch namespace1:=/nmea2tfpose_RTK1 namespace2:=/nmea2tfpose_RTK2 &
sleep 3
echo "currency : gnss_selector and gnss_localizer"

<<COMMENTOUT
#Set /config/ndt
rostopic pub __name:=config_ndt /config/ndt autoware_config_msgs/ConfigNDT "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
init_pos_gnss: 0
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
use_predict_pose: 1
error_threshold: 1.0
resolution: 2.0
step_size: 0.1
trans_epsilon: 0.01
max_iterations: 30
yaw_correct_deg: 0.0
position_y_correct: 0.0" -l &
sleep 1
echo "currency : /config/ndt"

#Set ndt
roslaunch lidar_localizer ndt_matching.launch method_type:=2 use_gnss:=1 use_odom:=False use_imu:=False imu_upside_down:=False imu_topic:=/imu_raw queue_size:=1 offset:=linear get_height:=True use_local_transform:=False sync:=False output_log_data:=False gnss_reinit_fitness:=500 &
sleep 3
echo "currency : ndt"
COMMENTOUT

#Set /config/can2odom
rostopic pub __name:=config_can2odom /config/can2odom autoware_config_msgs/ConfigCanOdometry "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
kmph_th: 0.1" -l &
sleep 1
echo "currency : /config/can2odom"

#Set can2odom
roslaunch autoware_connector can2odom.launch &
sleep 3
echo "currency : can2odom"

#Set /config/localizer_switch
rostopic pub __name:=config_localizer_switch /config/localizer_switch autoware_config_msgs/ConfigLocalizerSwitch "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
fusion_select: 1
localizer_check: 1
yaw_correction1: 0.0
yaw_correction2: 0.1" -l &
sleep 1
echo "currency : /config/localizer_switch"

#Set localizer_switch
roslaunch autoware_connector localizer_switch.launch base_link_pose_topic1:=/ndt_pose estimate_twist_topic1:=/ndt_estimate_twist  localizer_pose_topic1:=/ndt_localizer_pose alignment_mechanism1:=0 ndt_status_topic1:=/ndt_stat gnss_deviation_topic1:='' base_link_pose_topic2:=/RTK_gnss_pose estimate_twist_topic2:=/gnss_estimate_twist localizer_pose_topic2:=/gnss_localizer_pose alignment_mechanism2:=1 ndt_status_topic2:='' gnss_deviation_topic2:=/gnss_standard_deviation fusion_select:=1 &
sleep 3
echo "currency : localizer_switch"

#Set yollo3
roslaunch vision_darknet_detect vision_yolo3_detect.launch score_threshold:=0.6 nms_threshold:=0.45 image_src:=/image_raw network_definition_file:=/home/sit/lucid/weight_of_darknet/yolov3-signal.cfg pretrained_model_file:=/home/sit/lucid/weight_of_darknet/yolov3_final_new_moteghi_mix_60000.weights names_file:=/home/sit/lucid/weight_of_darknet/signal.name gpu_device_id:=0 detection_area_threshold:=70 camera_id:='' &
sleep 3
echo "currency : yolo3"

#Set signal_selector
roslaunch trafficlight_recognizer signal_selector.launch &
sleep3 
echo "currency : signal_selector"

#Set rviz
rosrun rviz rviz -d "${AUTOWARE_PATH}/src/autoware/utilities/autoware_launcher/profiles/quickstart/files/default.rviz" &
sleep 3
echo "currency : rviz"

<< COMMENTOUT
#Set /config/lane_rule
rostopic pub __name:=config_lane_rule /config/lane_rule autoware_config_msgs/ConfigLaneRule "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
acceleration: 0.325
stopline_search_radius: 1.0
number_of_zeros_ahead: 0
number_of_zeros_behind: 0
number_of_smoothing_count: 0
velocity_limit: 35" -l &
sleep 1
echo "currency : config_lane_rule"

#Set lane_rule
roslaunch lane_planner lane_rule_option.launch use_ll2:=False &
sleep 3
echo "currency : lane_rule"
COMMENTOUT

#Set /config/lane_stop
rostopic pub __name:=config_lane_stop /config/lane_stop autoware_config_msgs/ConfigLaneStop "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
manual_detection: false" -l &
sleep 1
echo "currency : config_lane_stop"

<< COMMENTOUT
#Set lane_stop
rosrun lane_planner lane_stop &
sleep 3
echo "currency : lane_stop"
COMMENTOUT

#Set /config/lane_select
rostopic pub __name:=config_lane_select /config/lane_select autoware_config_msgs/ConfigLaneSelect "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
distance_threshold_neighbor_lanes: 5.0
lane_change_interval: 10.0
lane_change_target_ratio: 5.0
lane_change_target_minimum: 10.0
vector_length_hermite_curve: 10.0" -l &
sleep 1
echo "currency : config_lane_select"

#Set lane_select
roslaunch lane_planner lane_select.launch search_closest_waypoint_minimum_dt:=5 &
sleep 3
echo "currency : lane_select"

#Set /config/waypoint_replanner
rostopic pub __name:=config_waypoint_replanner /config/waypoint_replanner autoware_config_msgs/ConfigWaypointReplanner "{multi_lane_csv: '', replanning_mode: false, use_decision_maker: false, velocity_max: 0.0,
  velocity_min: 0.0, accel_limit: 0.0, decel_limit: 0.0, radius_thresh: 0.0, radius_min: 0.0,
  resample_mode: false, resample_interval: 0.0, velocity_offset: 0, end_point_offset: 0,
  braking_distance: 0, replan_curve_mode: false, replan_endpoint_mode: false, overwrite_vmax_mode: false,
  realtime_tuning_mode: false}" -l &
sleep 1
echo "currency : /config/waypoint_replanner"

#Set waypoint_loader
roslaunch waypoint_maker waypoint_loader_show_id.launch load_csv:=True multi_lane_csv:=${WAYPOINT_FILE} replanning_mode:=False realtime_tuning_mode:=False resample_mode:=True resample_interval:=1.0 replan_curve_mode:=False overwrite_vmax_mode:=False replan_endpoint_mode:=True velocity_max:=20 radius_thresh:=20 radius_min:=6 velocity_min:=4 accel_limit:=0.5 decel_limit:=0.3 velocity_offset:=4 braking_distance:=5 end_point_offset:=1 use_decision_maker:=False &
sleep 3
echo "currency : waypoint_loader"

#Set waypoint_param_sender
roslaunch waypoint_maker waypoint_param_sender.launch &
sleep 3
echo "currency : waypoint_param_sendr"

#Set astar_avoid
roslaunch waypoint_planner astar_avoid.launch safety_waypoints_size:=100 update_rate:=10 costmap_topic:=semantics/costmap_generator/occupancy_grid enable_avoidance:=False avoid_waypoints_velocity:=10.0 avoid_start_velocity:=3.0 replan_interval:=2.0 search_waypoints_size:=50 search_waypoints_delta:=2 losest_search_size:=30 use_back:=False use_potential_heuristic:=True use_wavefront_heuristic:=False time_limit:=1000.0 robot_length:=4.5 robot_width:=1.75 robot_base2back:=1.0 minimum_turning_radius:=6.0 theta_size:=48 curve_weight:=1.2 reverse_weight:=2.00 lateral_goal_range:=0.5 longitudinal_goal_range:=2.0 angle_goal_range:=6.0 obstacle_threshold:=100 potential_weight:=10.0 distance_heuristic_weight:=1.0 &
sleep 3
echo "currency : astar_avoid"

#Set local_waypoint_adjustment
roslaunch waypoint_planner local_waypoint_adjustment.launch &
sleep 3
echo "currency : local_waypoint_adjustment"

<< COMMENTOUT
#Set /config/esr_tracker
rostopic pub /config/esr_tracker autoware_config_msgs/ConfigEsrTracker "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
judg_obj_pose_y: 1.5
obs_delete_time: 1
min_free_running_distance: 3
pose_variance_th: 3.5"  -l &
sleep 1
echo "/config/esr_tracker"

#Set mobileye_tracker
roslaunch waypoint_planner esr_tracker.launch &
sleep 3
echo "currency : esr_tracker"
COMMENTOUT

#Set /config/mobileye_tracker
rostopic pub /config/mobileye_tracker autoware_config_msgs/ConfigMobileyeTracker "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
judg_obj_pose_y: 1.5
obs_delete_time: 1.0
min_free_running_distance: 3.0
control_cycle: 10.0
expected_collision_time_th: 3.0
avoid_collision_corr1: 0.9
avoid_collision_corr2: 1.0"  -l &
sleep 1
echo "/config/mobileye_tracker"

#Set mobileye_tracker
roslaunch waypoint_planner mobileye_tracker.launch &
sleep 3
echo "currency : mobileye_tracker"

#Set /config/temporary_stopper
rostopic pub __name:=config_temporary_stopper /config/temporary_stopper autoware_config_msgs/ConfigTemporaryStopper "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
search_distance: 7
acceleration: -0.325
number_of_zeros_ahead: 0
number_of_zeros_behind: 0
stop_speed_threshold: 0.1
fixed_velocity: 0.0
velocity_limit: 35" -l &
sleep 1
echo "currency : /config/temporary_stopper"

#Set temporary_stopper
roslaunch waypoint_planner temporary_stopper.launch &
sleep 3
echo "currency : temporary_stopper"

#Set /config/velocity_set
rostopic pub __name:=config_velocity_set /config/velocity_set autoware_config_msgs/ConfigVelocitySet "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
stop_distance_obstacle: 7.0
stop_distance_stopline: 5.0
detection_range: 0.8
threshold_points: 8
detection_height_top: 0.1
detection_height_bottom: -1.5
deceleration_obstacle: 0.7
deceleration_stopline: 0.8
velocity_change_limit: 40
deceleration_range: 0.0
temporal_waypoints_size: 150" -l &
sleep 1
echo "currency : /config/velocity_set"

#Set velocity_set
roslaunch waypoint_planner velocity_set_option.launch use_crosswalk_detection:=False points_topic:=points_no_ground velocity_offset:=1.2 decelerate_vel_min:=1.3 remove_points_upto:=2.3 enable_multiple_crosswalk_detection:=False stop_distance_obstacle:=10.0 stop_distance_stopline:=5.0 detection_range:=1.3 points_threshold:=10 detection_height_top:=0.2 detection_height_bottom:=-1.7 deceleration_obstacle:=0.8 deceleration_stopline:=0.6 velocity_change_limit:=9.972 deceleration_range:=0 temporal_waypoints_size:=100 use_ll2:=False &
sleep 3
echo "currency : velocity_set"

<< COMMENTOUT
#Set /config/waypoint_follower
rostopic pub __name:=config_waypoint_follower /config/waypoint_follower autoware_config_msgs/ConfigWaypointFollower "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
param_flag: 0
velocity: 5.0
lookahead_distance: 4.0
lookahead_ratio: 2.0
minimum_lookahead_distance: 6.0
displacement_threshold: 0.0
relative_angle_threshold: 0.0" -l &
sleep 1
echo "currency : /config/waypoint_follower"

#Set pure_pursuit
roslaunch pure_pursuit pure_pursuit.launch is_linear_interpolation:=True publishes_for_steering_robot:=True add_virtual_end_waypoints:=False const_lookahead_distance:=4.0 const_velocity:=5.0 lookahead_ratio:=2.0 minimum_lookahead_distance:=6.0 &
sleep 3
echo "currency : pure_pursuit"
COMMENTOUT

#Set mpc_follower
roslaunch mpc_follower mpc_follower.launch ns:=/ show_debug_info:=False ctrl_period:=0.01 traj_resample_dist:=0.1 enable_yaw_recalculation:=True admisible_position_error:=5.0 admisible_yaw_error_deg:=90.0 enable_path_smoothing:=True path_smoothing_times:=1 path_filter_moving_ave_num:=35 curvature_smoothing_num:=35 steering_lpf_cutoff_hz:=3.0 qp_solver_type:=unconstraint_fast qpoases_max_iter:=500 vehicle_model_type:=kinematics mpc_prediction_horizon:=70 mpc_prediction_sampling_time:=0.1 mpc_weight_lat_error:=0.1 mpc_weight_heading_error:=0.0 mpc_weight_heading_error_squared_vel_coeff:=0.3 mpc_weight_steering_input:=1.0 mpc_weight_steering_input_squared_vel_coeff:=0.25 mpc_weight_lat_jerk:=0.0 mpc_weight_terminal_lat_error:=10.0 mpc_weight_terminal_heading_error:=0.1 mpc_zero_ff_steer_deg:=2.0 delay_compensation_time:=0.0 vehicle_model_steer_tau:=0.6 vehicle_model_wheelbase:=4.4 steer_lim_deg:=35.0 steering_gear_ratio:=20.0 &
sleep 3
echo "currency : mpc_follower"

#Set /config/cmd_selector
rostopic pub __name:=config_cmd_selector /config/cmd_selector autoware_config_msgs/ConfigCmdSelector "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
cmd_select: '1'" -l &
sleep 1
echo "currency : /config/cmd_selector"

#Set cmd_selector
roslaunch cmd_selector cmd_selector.launch &
sleep 3
echo "currency : cmd_selector"

#Set /config/twist_filter
rostopic pub __name:=config_twist_filter /config/twist_filter autoware_config_msgs/ConfigTwistFilter "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
use_decision_maker: false
lateral_accel_limit: 5  
lateral_jerk_limit: 5  
lowpass_gain_linear_x: 0.0
lowpass_gain_angular_z: 0.0
lowpass_gain_steering_angle: 0.0" -l &

#Set twist_filter
roslaunch twist_filter twist_filter.launch wheel_base:=4.4 lateral_accel_limit:=5.0 lateral_jerk_limit:=5.0 lowpass_gain_linear_x:=0.0 lowpass_gain_angular_z:=0.0 lowpass_gain_steering_angle:=0.0 loop_rate:=30.0 use_decision_maker:=False &

#Set /config/lookahead_ratio_magn
rostopic pub /config/lookahead_ratio_magn autoware_config_msgs/ConfigLookAheadRatioMagn "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
max_magn: 1.0
min_magn: 1.0
max_distance: 0.0
min_distance: 0.0" -l &
sleep 1
echo "currency : /config/lookahead_ratio_magn"

#Set lookahead_ratio_magn
roslaunch lookahead_ratio_magn lookahead_ratio_magn.launch &
sleep
echo "currency : lookahead_ratio_magn"

#Set mileage_publisher
roslaunch mileage_publisher mileage_publisher.launch &
sleep
echo "currency : mileage_publisher"

#Set /config/microbus_can
rostopic pub /config/microbus_can autoware_config_msgs/ConfigMicroBusCan "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
use_position_checker: false
velocity_limit: 70
velocity_stop_th: 0.0
accel_max_i: 1250
brake_max_i: 520
k_accel_p_velocity: 15.0
k_accel_i_velocity: 0.16
k_accel_d_velocity: 0.1
k_brake_p_velocity: 0.32
k_brake_i_velocity: 0.215
k_brake_d_velocity: 0.3
k_accel_p_acceleration: 0.12
k_accel_i_acceleration: 0.0
k_accel_d_acceleration: 0.0
k_brake_p_acceleration: 0.12
k_brake_i_acceleration: 0.0
k_brake_d_acceleration: 0.0
k_accel_p_distance: 0.0
k_accel_i_distance: 0.0
k_accel_d_distance: 0.0
k_brake_p_distance: 0.5
k_brake_i_distance: 0.0
k_brake_d_distance: 0.0
steer_max_i: 100
k_steer_p_distance: 130
k_steer_i_distance: 0.0
k_steer_d_distance: 0.0
pedal_stroke_center: 0
pedal_stroke_max: 840
pedal_stroke_min: -500
brake_stroke_stopping_med: -300
accel_stroke_offset: 10
brake_stroke_offset: -10
gnss_lat_limit: 0.12
gnss_lon_limit: 0.12
gnss_alt_limit: 0.15
gnss_yaw_limit: 0.1
acceptable_velocity_variation: 0
ndt_gnss_min_distance_limit: 0.3
ndt_gnss_max_distance_limit: 0.5
ndt_gnss_angle_limit: 10
steer_speed_limit1: 100
steer_speed_limit2: 75
check_distance_th: 1.1
check_angular_th: 20
stopper_distance1: 30
stopper_distance2: 8
stopper_distance3: 2
use_lane_left: False
use_lane_right: False
lane_th_left: -5
lane_th_right: 5
accel_stroke_step_max: 1.5
accel_stroke_step_min: 0.5
brake_stroke_step_max: 2
brake_stroke_step_min: 1
accel_stroke_adjust_th: 15
brake_stroke_adjust_th: 15
pedal_center_voltage: 1000" -l &
sleep 1
echo "currency : /config/microbus_can"

#Set kvaser_microbus_stroke
roslaunch vehicle_socket kvaser_microbus.launch kvaser_channel:=1 use_velocity_topic:=1 &
sleep 3
echo "currency : kvaser_microbus_stroke"

#Set microbus_interface
roslaunch microbus_interface microbus_interface.launch &
sleep 3
echo "currency : microbus_interface"

#Set rviz_restart
rosrun rviz_restart rviz_restart &
sleep 3
echo "currency : rviz_restart"
