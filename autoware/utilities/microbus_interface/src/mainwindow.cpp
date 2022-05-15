#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialog_driving_adjustment.h"
#include "ui_dialog_driving_adjustment.h"
#include "dialog_popup_signal.h"
#include "ui_dialog_popup_signal.h"

std::string stringReplace(const std::string &string, const char from, const char to)
{
	std::string new_str;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, from))
		new_str += token + to;
	return new_str.substr(0, new_str.size()-1);
}

std::string rostime2date(ros::Time nowtime)
{
	std::chrono::system_clock::time_point p = std::chrono::system_clock::from_time_t((time_t)nowtime.sec);
	std::time_t t = std::chrono::system_clock::to_time_t(p);
	struct tm date;
	localtime_r(&t, &date);
	std::stringstream ss;
	ss << 1900 + date.tm_year << "-" << date.tm_mon+1 << "-" << date.tm_mday << "_" << date.tm_hour << "-" << date.tm_min << "-" << std::fixed << std::setprecision(4) <<(double)date.tm_sec + nowtime.nsec * 1E-9;
	return ss.str();
}

double euclideanDistanceXY(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
	double x1 = p1.x,  x2 = p2.x;
	double y1 = p1.y,  y2 = p2.y;
	double xd = x1 - x2,  yd = y1 - y2;
	return std::sqrt(xd*xd + yd*yd);
}

int existFile(const char* path)
{
	//pathが存在するか
	struct stat st;
	if (stat(path, &st) != 0) {
		return 0;
	}

	// ファイルかどうか
	 return (st.st_mode & S_IFMT) == S_IFREG;
}

std::vector<std::string> split(const std::string &string, const char sep)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, sep))
	str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

void killWaypointsNode()
{
	system("rosnode kill /waypoint_loader_show_id &");
	system("rosnode kill /waypoint_replanner &");
	system("rosnode kill /waypoint_marker_publisher_show_id &");
	//ros::Rate rate(1.0);
	//rate.sleep();
}

/*void MainWindow::runWaypointsNode(std::string branch)
{
	std::string launch_str = ros::package::getPath("waypoint_maker") + "/launch/" + branch + ".launch";
	if(existFile(launch_str.c_str()) == false)
	{
		QMessageBox msgBox(this);
		std::string error_str = "経路launchファイルがありません\n" + launch_str;
		msgBox.setText(error_str.c_str());
		msgBox.setWindowTitle("error");
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.setDefaultButton(QMessageBox::Ok);
		msgBox.exec();
	}
	else
	{
		std::string cmd = "roslaunch waypoint_maker " + branch + ".launch &";
		std::cout << "cmd : " << cmd << std::endl;
		system(cmd.c_str());
	}
}*/

MainWindow::MainWindow(ros::NodeHandle nh, ros::NodeHandle p_nh, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	localizer_select_(-1),
	signal_red_green_time_(0),
	signal_green_yellow_time_(0),
	signal_yellow_red_time_(0),
	signal_red_green_time2_(0),
	signal_change_time_(0),
	period_signal_takeover_(false),
	automode_mileage_(0),
	//load_name_count_(1),
	use_specified_cmd_(false),
	steer_plus_sum_(0.0),
	use_steer_plus_sum_(false),
	steer_plus_sum_count_(0),
	read_safety_waypoints_(false),
	panel_signal_msg_(0),
	panel_view_front_(0),
	panel_view_back_(0),
	prev_blinker_(-1),
	oncoming_stop_id_(-1),
	rosbag_write_flag_(false)
{
	ui->setupUi(this);

	nh_.param<double>("/vehicle_info/wheelrad_to_steering_can_value_left", wheelrad_to_steering_can_value_left_, 25009.6727514125);
	nh_.param<double>("/vehicle_info/wheelrad_to_steering_can_value_right", wheelrad_to_steering_can_value_right_, 26765.9140133745);

	light_color_.traffic_light = MainWindow::TRAFFIC_LIGHT_UNKNOWN;
	ui->gb_shift->setVisible(false);
	ui->gb_velocity->setVisible(false);

	palette_drive_mode_ok_ = ui->tx_drive_mode->palette();
	palette_steer_mode_ok_ = ui->tx_steer_mode->palette();
	palette_drive_clutch_connect_ = ui->tx_drive_clutch->palette();
	palette_steer_clutch_connect_ = ui->tx_steer_clutch->palette();
	palette_distance_angular_ok_ = ui->tx_distance_check->palette();
	palette_localizer_select_ok_ = ui->tx_localizer_select->palette();
	palette_gnss_deviation_ok_ = ui->tx_lat->palette();
	palette_score_ok_ = ui->tx_ndt_score->palette();
	palette_lb_normal_ = ui->lb2_ndt->palette();
	palette_signal_text_unknown_ = ui->tx2_signal_color->palette();
	palette_auto_check_ok_ = ui->tx4_auto_ok->palette();
	palette_logbt_on_ = ui->bt2_log_stop->palette();
	palette_drive_mode_error_ = palette_drive_mode_ok_;
	palette_steer_mode_error_ = palette_steer_mode_ok_;
	palette_drive_clutch_cut_ = palette_drive_clutch_connect_;
	palette_steer_clutch_cut_ = palette_steer_clutch_connect_;
	palette_distance_angular_error_ = palette_distance_angular_ok_;
	palette_localizer_select_error_ = palette_localizer_select_ok_;
	palette_gnss_deviation_error_ = palette_gnss_deviation_ok_;
	palette_score_error_ = palette_score_ok_;
	palette_current_localizer_ = palette_localizer_select_ok_;
	palette_lb_localize_ = palette_lb_normal_;
	palette_signal_text_green_ = palette_signal_text_unknown_;
	palette_signal_text_red_ = palette_signal_text_unknown_;
	palette_stop_line_non_ = ui->tx2_stopD->palette();
	palette_auto_check_error_ = palette_auto_check_ok_;
	palette_logbt_off_ = ui->bt2_log_write->palette();
	palette_drive_mode_error_.setColor(QPalette::Base, QColor("#FFFF00"));
	palette_steer_mode_error_.setColor(QPalette::Base, QColor("#FFFF00"));
	palette_drive_clutch_cut_.setColor(QPalette::Base, QColor("#00FF00"));
	palette_steer_clutch_cut_.setColor(QPalette::Base, QColor("#00FF00"));
	palette_distance_angular_error_.setColor(QPalette::Base, QColor("#FFFF00"));
	palette_distance_angular_ok_.setColor(QPalette::Base, QColor("#00FFFF"));
	palette_localizer_select_error_.setColor(QPalette::Base, QColor("#FF0000"));
	palette_gnss_deviation_error_.setColor(QPalette::Base, QColor("#FFFF00"));
	palette_score_error_.setColor(QPalette::Base, QColor("#FF0000"));
	palette_current_localizer_.setColor(QPalette::Base, QColor("#00FF00"));
	palette_lb_localize_.setColor(QPalette::Base, QColor("#00FF00"));
	palette_signal_text_green_.setColor(QPalette::Base, QColor("#00FF00"));
	palette_signal_text_red_.setColor(QPalette::Base, QColor("#FF0000"));
	palette_period_signal_takeover_.setColor(QPalette::Text, QColor("#FF0000"));
	palette_stop_line_middle_.setColor(QPalette::Base, QColor("#FF0000"));
	palette_stop_line_stop_.setColor(QPalette::Base, QColor("#0000A0"));
	palette_auto_check_ok_.setColor(QPalette::Base, QColor("#FFFF00"));
	palette_auto_check_error_.setColor(QPalette::Base, QColor("#FF0000"));
	ui->tx2_period_signal_takeover->setPalette(palette_period_signal_takeover_);
	ui->lb2_ndt->setPalette(palette_lb_localize_);
	ui->lb2_ekf->setPalette(palette_lb_localize_);
	ui->lb2_gnss->setPalette(palette_lb_localize_);

	connect(ui->bt_emergency_clear, SIGNAL(clicked()), this, SLOT(publish_emergency_clear()));
	connect(ui->bt_drive_mode_manual, SIGNAL(clicked()), this, SLOT(publish_Dmode_manual()));
	connect(ui->bt_drive_mode_program, SIGNAL(clicked()), this, SLOT(publish_Dmode_program()));
	connect(ui->bt_drive_control_mode_velocity, SIGNAL(clicked()), this, SLOT(publish_Dmode_velocity()));
	connect(ui->bt_drive_control_mode_stroke, SIGNAL(clicked()), this, SLOT(publish_Dmode_stroke()));
	connect(ui->bt_drive_input_mode_direct, SIGNAL(clicked()), this, SLOT(publish_Dmode_input_direct()));
	connect(ui->bt_drive_input_mode_autoware, SIGNAL(clicked()), this, SLOT(publish_Dmode_input_auto()));
	connect(ui->bt_steer_mode_manual, SIGNAL(clicked()), this, SLOT(publish_Smode_manual()));
	connect(ui->bt_steer_mode_program, SIGNAL(clicked()), this, SLOT(publish_Smode_program()));
	connect(ui->bt_steer_input_mode_direct, SIGNAL(clicked()), this, SLOT(publish_Smode_input_direct()));
	connect(ui->bt_steer_input_mode_autoware, SIGNAL(clicked()), this, SLOT(publish_Smode_input_auto()));
	connect(ui->bt_drive_clutch_connect, SIGNAL(clicked()), this, SLOT(publish_drive_clutch_connect()));
	connect(ui->bt_drive_clutch_cut, SIGNAL(clicked()), this, SLOT(publish_drive_clutch_cut()));
	connect(ui->bt_steer_clutch_connect, SIGNAL(clicked()), this, SLOT(publish_steer_clutch_connect()));
	connect(ui->bt_steer_clutch_cut, SIGNAL(clicked()), this, SLOT(publish_steer_clutch_cut()));
	connect(ui->bt_error_text_reset, SIGNAL(clicked()), this, SLOT(click_error_text_reset()));
	connect(ui->bt_blinker_right_on, SIGNAL(clicked()), this, SLOT(publish_blinker_right()));
	connect(ui->bt_blinker_left_on, SIGNAL(clicked()), this, SLOT(publish_blinker_left()));
	connect(ui->bt_blinker_right_off, SIGNAL(clicked()), this, SLOT(publish_blinker_stop()));
	connect(ui->bt_blinker_left_off, SIGNAL(clicked()), this, SLOT(publish_blinker_stop()));
	connect(ui->bt2_blinker_right, SIGNAL(clicked()), this, SLOT(publish_blinker_right()));
	connect(ui->bt2_blinker_left, SIGNAL(clicked()), this, SLOT(publish_blinker_left()));
	connect(ui->bt2_blinker_stop, SIGNAL(clicked()), this, SLOT(publish_blinker_stop()));
	connect(ui->bt2_error_clear, SIGNAL(clicked()), this, SLOT(click_error_text_reset()));
	connect(ui->bt2_drive_on,SIGNAL(clicked()), this, SLOT(publish_Dmode_program()));
	connect(ui->bt2_steer_on,SIGNAL(clicked()), this, SLOT(publish_Smode_program()));
	connect(ui->bt2_drive_off,SIGNAL(clicked()), this, SLOT(publish_Dmode_manual()));
	connect(ui->bt2_steer_off,SIGNAL(clicked()), this, SLOT(publish_Smode_manual()));
	connect(ui->cb_use_localizer_safety, SIGNAL(clicked()), this, SLOT(publish_use_safety_localizer()));
	connect(ui->cb_error_check, SIGNAL(clicked()), this, SLOT(publish_use_error_check()));
	connect(ui->bt2_log_write, SIGNAL(clicked()), this, SLOT(publish_log_write()));
	connect(ui->bt2_log_stop, SIGNAL(clicked()), this, SLOT(publish_log_stop()));
	connect(ui->bt3_signal_time, SIGNAL(clicked()), this, SLOT(click_signal_time()));
	connect(ui->bt3_signal_time_clear, SIGNAL(clicked()), this, SLOT(click_signal_time_clear()));
	connect(ui->bt2_log_folder, SIGNAL(clicked()), this, SLOT(click_log_folder()));
	connect(ui->bt4_nextA, SIGNAL(clicked()), this, SLOT(click_load_nextA()));
	connect(ui->bt4_backA, SIGNAL(clicked()), this, SLOT(click_load_backA()));
	connect(ui->bt4_nextB, SIGNAL(clicked()), this, SLOT(click_load_nextB()));
	connect(ui->bt4_backB, SIGNAL(clicked()), this, SLOT(click_load_backB()));
	connect(ui->bt4_loop_next, SIGNAL(clicked()), this, SLOT(click_auto_change_loop_next()));
	connect(ui->bt4_loop_back, SIGNAL(clicked()), this, SLOT(click_auto_change_loop_back()));
	connect(ui->bt5_can_send, SIGNAL(clicked()), this, SLOT(publish_specified_speed()));
	connect(ui->bt5_can_stop, SIGNAL(clicked()), this, SLOT(publish_specified_speed_stop()));
	connect(ui->bt5_velocity_plus1, SIGNAL(clicked()), this, SLOT(click_specified_speed_plus1()));
	connect(ui->bt5_velocity_minus1, SIGNAL(clicked()), this, SLOT(click_specified_speed_minus1()));
	connect(ui->bt5_velocity_plus5, SIGNAL(clicked()), this, SLOT(click_specified_speed_plus5()));
	connect(ui->bt5_velocity_minus5, SIGNAL(clicked()), this, SLOT(click_specified_speed_minus5()));
	connect(ui->bt5_deg_plus1, SIGNAL(clicked()), this, SLOT(click_specified_deg_plus1()));
	connect(ui->bt5_deg_minus1, SIGNAL(clicked()), this, SLOT(click_specified_deg_minus1()));
	connect(ui->bt5_deg_plus5, SIGNAL(clicked()), this, SLOT(click_specified_deg_plus5()));
	connect(ui->bt5_deg_minus5, SIGNAL(clicked()), this, SLOT(click_specified_deg_minus5()));
	connect(ui->bt5_deg_plus_any, SIGNAL(clicked()), this, SLOT(click_specified_deg_plus_any()));
	connect(ui->bt5_deg_minus_any, SIGNAL(clicked()), this, SLOT(click_specified_deg_minus_any()));
	connect(ui->bt2_steer_plus, SIGNAL(clicked()), this, SLOT(click_steer_plus()));
	connect(ui->bt2_steer_plus_ave, SIGNAL(clicked()), this, SLOT(click_steer_plus_ave()));
	connect(ui->bt2_steer_plus_ave_stop, SIGNAL(clicked()), this, SLOT(click_steer_plus_ave_stop()));
	connect(ui->bt2_steer_plus_P10, SIGNAL(clicked()), this, SLOT(click_steer_P10()));
	connect(ui->bt2_steer_plus_M10, SIGNAL(clicked()), this, SLOT(click_steer_M10()));
	connect(ui->ch5_use_auto_change, SIGNAL(clicked(bool)), this, SLOT(check_use_auto_chenge(bool)));
	connect(ui->ch5_use_auto_log, SIGNAL(clicked(bool)), this, SLOT(check_use_auto_log(bool)));
	connect(ui->slider_can_send_velocity, SIGNAL(valueChanged(int)), this, SLOT(slide_specified_speed(int)));
	connect(ui->slider_can_send_deg, SIGNAL(valueChanged(int)), this, SLOT(slide_specified_deg(int)));
	connect(ui->bt4_rviz_restart, SIGNAL(clicked()), this, SLOT(click_rviz_restart()));
	connect(ui->bt3_arena_gain_send, SIGNAL(clicked()), this, SLOT(click_arena_gain_send()));
	connect(ui->bt_auto_shutdown, SIGNAL(clicked()), this, SLOT(click_auto_shutdown()));
	connect(ui->list_read_launch, SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)), this, SLOT(currentItemchange_list_read_launch(QListWidgetItem*,QListWidgetItem*)));
	connect(ui->bt4_dialog_driving_adjustment, SIGNAL(clicked()), this, SLOT(click_call_dialog_driving_adjustment()));
	connect(ui->bt_steer_proofreading_main, SIGNAL(clicked()), this, SLOT(click_steer_proofreading_main()));
	connect(ui->bt_steer_proofreading_sub, SIGNAL(clicked()), this, SLOT(click_steer_proofreading_base()));
	connect(ui->bt4_rosbag, SIGNAL(clicked()), this, SLOT(click_rosbag()));
	connect(ui->sb_target_deceleration, SIGNAL(valueChanged(double)), this, SLOT(car_target_deceleration_change(double)));
	connect(ui->bt_track_excess_acc, SIGNAL(clicked()), this, SLOT(click_track_excess_acc()));
	connect(ui->bt_track_excess_stop, SIGNAL(clicked()), this, SLOT(click_track_excess_stop()));
	connect(ui->bt_yure, SIGNAL(clicked()), this, SLOT(click_yure()));

	nh_ = nh;  private_nh_ = p_nh;

	pub_unlock_ = nh_.advertise<std_msgs::Empty>("/microbus/first_lock_release", 1);
	pub_drive_mode_ = nh_.advertise<std_msgs::Bool>("/microbus/drive_mode_send", 1);
	pub_drive_control_ = nh_.advertise<std_msgs::Int8>("/microbus/drive_control", 1);
	pub_steer_mode_ = nh_.advertise<std_msgs::Bool>("/microbus/steer_mode_send", 1);
	pub_drive_input_ = nh_.advertise<std_msgs::Bool>("/microbus/input_drive_flag", 1);
	pub_steer_input_ = nh_.advertise<std_msgs::Bool>("/microbus/input_steer_flag", 1);
	pub_drive_clutch_ = nh_.advertise<std_msgs::Bool>("/microbus/drive_clutch", 1);
	pub_steer_clutch_ = nh_.advertise<std_msgs::Bool>("/microbus/steer_clutch", 1);
	pub_blinker_left_ = nh_.advertise<std_msgs::Bool>("/microbus/blinker_left", 1);
	pub_blinker_right_ = nh_.advertise<std_msgs::Bool>("/microbus/blinker_right", 1);
	pub_blinker_stop_ = nh_.advertise<std_msgs::Bool>("/microbus/blinker_stop", 1);
	pub_error_lock_ = nh_.advertise<std_msgs::Bool>("/microbus/interface_lock", 1);
	pub_use_safety_localizer_ = nh_.advertise<std_msgs::Bool>("/microbus/use_safety_localizer", 1, true);
	pub_log_write_ = nh_.advertise<std_msgs::Bool>("/microbus/log_on", 1);
	pub_log_folder_ = nh_.advertise<std_msgs::String>("/microbus/log_folder", 1, true);
	pub_vehicle_cmd_ = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1);
	pub_use_error_check_ = nh_.advertise<std_msgs::Bool>("/microbus/error_check", 1, true);
	pub_steer_plus_ = nh_.advertise<std_msgs::Int16>("/microbus/steer_plus", 1, true);
	pub_auto_log_ = nh_.advertise<std_msgs::Bool>("/microbus/auto_log_write", 1, true);
	pub_arena_gain_ = nh_.advertise<std_msgs::Float64>("/arena/gain", 1);
	pub_auto_route_list_ = nh_.advertise<autoware_msgs::AutoRouteList>("/auto_route_list", 1, true);
	pub_auto_route_increment_ = nh_.advertise<std_msgs::Bool>("/auto_route_increment", 1);
	pub_auto_route_loop_next_ = nh_.advertise<std_msgs::Bool>("/auto_route_loop_next", 1);
	pub_steer_sub_interface_correction_ = nh_.advertise<autoware_msgs::SteerSubInterfaceCorrection>("/microbus/steer_sub_interface_correction", 1);
	pub_steer_sub_interface_correction_write_end_ = nh_.advertise<std_msgs::Bool>("/microbus/steer_sub_interface_correction_write_end", 1);
	pub_change_max_speed_ = nh_.advertise<std_msgs::Float64>("/microbus/change_max_speed", 1);
	pub_panel_view_ = nh_.advertise<std_msgs::UInt16>("/panel_view/write", 1);
	pub_panel_view_reset_ = nh_.advertise<std_msgs::Empty>("/panel_view/write_reset", 1);
	pub_popup_return_ = nh_.advertise<autoware_msgs::InterfacePopupReturn>("/interface_popup_return", 1);
	pub_oncoming_permission_ = nh_.advertise<std_msgs::UInt32>("/oncoming_permission", 1);
	pub_record_topic_list_ = nh_.advertise<std_msgs::String>("/record_topic_list", 1, true);
	pub_car_target_deceleration_ = nh_.advertise<std_msgs::Float64>("/car_target_deceleration", 1, true);

	sub_can501_ = nh_.subscribe("/microbus/can_receive501", 10, &MainWindow::callbackCan501, this);
	sub_can502_ = nh_.subscribe("/microbus/can_receive502", 10, &MainWindow::callbackCan502, this);
	sub_can503_ = nh_.subscribe("/microbus/can_receive503", 10, &MainWindow::callbackCan503, this);
	sub_can_status_ = nh_.subscribe("/microbus/can_sender_status", 10, &MainWindow::callbackCanStatus, this);
	sub_distance_angular_check_ = nh_.subscribe("/difference_to_waypoint_distance", 10, &MainWindow::callbackDistanceAngularCheck, this);
	sub_distance_angular_check_ndt_ = nh_.subscribe("/difference_to_waypoint_distance_ndt", 10, &MainWindow::callbackDistanceAngularCheckNdt, this);
	sub_distance_angular_check_ekf_ = nh_.subscribe("/difference_to_waypoint_distance_ekf", 10, &MainWindow::callbackDistanceAngularCheckEkf, this);
	sub_distance_angular_check_gnss_ = nh_.subscribe("/difference_to_waypoint_distance_gnss", 10, &MainWindow::callbackDistanceAngularCheckGnss, this);
	sub_config_ = nh_.subscribe("/config/microbus_can", 10, &MainWindow::callbackConfig, this);
	sub_localizer_select_ = nh_.subscribe("/localizer_select_num", 10, &MainWindow::callbackLocalizerSelect, this);
	sub_localizer_match_stat_ = nh_.subscribe("/microbus/localizer_match_stat", 10, &MainWindow::callbackLocalizerMatchStat, this);
	sub_can_velocity_param_ = nh_.subscribe("/microbus/velocity_param", 10, &MainWindow::callbackCanVelocityParam, this);
	sub_stopper_distance_ = nh_.subscribe("/stopper_distance", 10, &MainWindow::callbackStopperDistance, this);
	sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &MainWindow::callbackWaypointParam, this);
	sub_imu_ = nh_.subscribe("/gnss_imu", 10, &MainWindow::callbackImu, this);
	sub_gnss_pose_ = nh_.subscribe("/gnss_pose", 10, &MainWindow::callbackGnssPose, this);
	sub_gnss_deviation_ = nh_.subscribe("/gnss_standard_deviation", 10, &MainWindow::callbackGnssDeviation, this);
	sub_ndt_stat_ = nh_.subscribe("/ndt_stat", 10, &MainWindow::callbackNdtStat, this);
	sub_gnss_stat_ = nh_.subscribe("/gnss_rtk_stat", 10, &MainWindow::callbackGnssStat, this);
	sub_ndt_stat_string_ = nh.subscribe("/ndt_monitor/ndt_status", 10 , &MainWindow::callbackNdtStatString, this);
	sub_stroke_routine_ = nh.subscribe("/microbus/stroke_routine", 10 , &MainWindow::callbackStrokeRoutine, this);
	sub_mobileye_frame_ = nh.subscribe("/can_tx", 10 , &MainWindow::callbackMobileyeCan, this);
	sub_gnss_time_ = nh.subscribe("/gnss_time", 10 , &MainWindow::callbackGnssTime, this);
	sub_light_color_ = nh.subscribe("/light_color", 10 , &MainWindow::callbackLightColor, this);
	sub_signal_change_time_ = nh.subscribe("/signal_change_time", 10 , &MainWindow::callbackSignalChangeTime, this);
	sub_period_signal_takeover_ = nh.subscribe("/period_signal_takeover", 10 , &MainWindow::callbackPeriodSignalTakeover, this);
	sub_automode_mileage_ = nh.subscribe("/way_current_distance_all", 10 , &MainWindow::callbackAutomodeMileage, this);
	sub_vehicle_cmd_ = nh.subscribe("/vehicle_cmd", 10 , &MainWindow::callbackVehicleCmd, this);
	sub_cmd_select_ = nh.subscribe("/cmd_selector/select", 10 , &MainWindow::callbackCmdSelect, this);
	sub_load_name_ = nh.subscribe("/load_name", 10 , &MainWindow::callbackLoadName, this);
	sub_base_waypoints_ = nh.subscribe("/lane_waypoints_array", 10 , &MainWindow::callbackBaseWaypoints, this);
	sub_current_velocity_ = nh.subscribe("/current_velocity", 10 , &MainWindow::callbackCurrentVelocity, this);
	sub_acc_ = nh.subscribe("/microbus/acc", 10 , &MainWindow::callbackAcc, this);
	sub_auto_route_loop_count_ = nh.subscribe("/auto_route_loop_count", 10 , &MainWindow::callbackAutoRouteLoopCount, this);
	sub_safety_waypoints_ = nh.subscribe("/safety_waypoints", 10 , &MainWindow::callbackSafetyWaypoints, this);
	sub_steer_correction_write_return_ = nh.subscribe("/microbus/steer_correction_write_return", 10 , &MainWindow::callbackSteerCorrectionWriteReturn, this);
	sub_panel_read_ = nh.subscribe("/panel_view/read", 10 , &MainWindow::callbackPanelRead, this);
	sub_popup_signal_ = nh.subscribe("/interface_popup_signal", 10 , &MainWindow::callbackPopupSignal, this);
	sub_oncoming_obs_ = nh.subscribe("/oncoming_obs", 10 , &MainWindow::callbackOncomingObs, this);
	sub_steer_proofreading_main_ = nh.subscribe("/steer_proofreading/main", 10 , &MainWindow::callbackSteerProofreadingMain, this);
	sub_steer_proofreading_base_ = nh.subscribe("/steer_proofreading/base", 10 , &MainWindow::callbackSteerProofreadingBase, this);
	sub_front_mobileye_ = nh.subscribe("/mobileye_tracker/front_mobileye", 10 , &MainWindow::callbackFrontMobileye, this);
	sub_mobileye_cmd_param_ = nh.subscribe("/mobileye_tracker/cmd_param", 10 , &MainWindow::callbackMobileyeCmdParam, this);
	sub_tracking_type_ = nh.subscribe("/mobileye_tracker/track_pattern", 10 , &MainWindow::callbackTrackingType, this);
	sub_config_voxel_grid_filter_ = nh.subscribe("/config/voxel_grid_filter", 10 , &MainWindow::callbackVoxelGirdFilter, this);

	can_status_.angle_limit_over = can_status_.position_check_stop = true;
	error_text_lock_ = false;
	distance_angular_check_.baselink_distance = 10000;
	distance_angular_check_.baselink_angular = 180;
	distance_angular_check_ndt_.baselink_distance = 10000;
	distance_angular_check_ndt_.baselink_angular = 180;
	distance_angular_check_ekf_.baselink_distance = 10000;
	distance_angular_check_ekf_.baselink_angular = 180;
	distance_angular_check_gnss_.baselink_distance = 10000;
	distance_angular_check_gnss_.baselink_angular = 180;
	stopper_distance_.distance = -1;

	publish_use_safety_localizer();

	//非表示
	ui->lb2_jurk->setVisible(false);
	ui->tx2_jurk->setVisible(false);

	//無効
	//ui->bt4_backB->setEnabled(false);
	//ui->bt4_nextB->setEnabled(false);
	//ui->bt4_backA->setEnabled(false);

	ui->tx4_load_name->setAlignment(Qt::AlignCenter);

	ros::Time nowtime = ros::Time::now();
	timer_error_lock_ = nowtime;
	current_velocity_.header.stamp = nowtime;

	//保存されているlog_folderのパスを取得
	{
		std::string param_path = ros::package::getPath("runtime_manager") + "/param";
		std::ifstream ifs_param(param_path, std::ios_base::in);
		int read_launch_select = 0;//自動経路選択
		if(ifs_param)
		{
			std::getline(ifs_param, log_folder_);
			std_msgs::String str;
			str.data = log_folder_;
			pub_log_folder_.publish(str);

			std::string str_read_launch_select;
			std::getline(ifs_param, str_read_launch_select);
			read_launch_select = atoi(str_read_launch_select.c_str());

			std::string str_prev_steer_center;
			std::getline(ifs_param, str_prev_steer_center);
			ui->li2_steer_plus_interface->setText(str_prev_steer_center.c_str());

			std::string str_vehicle_distance;
			std::getline(ifs_param, str_vehicle_distance);
			double car_target_deceleration = atof(str_vehicle_distance.c_str());
			ui->sb_target_deceleration->setValue(car_target_deceleration);
			std_msgs::Float64 msg_car_target_deceleration;
			msg_car_target_deceleration.data = car_target_deceleration;
			pub_car_target_deceleration_.publish(msg_car_target_deceleration);

			ifs_param.close();

			int16_t steer_center_val = atoi(str_prev_steer_center.c_str());
			std_msgs::Int16 msg_steer_center_val;
			msg_steer_center_val.data = steer_center_val;
			pub_steer_plus_.publish(msg_steer_center_val);
		}

		//ui->tx4_auto_ok->setAlignment(Qt::AlignJustify);

		for(int i=0; i<select_launch_text_.size(); i++)
			ui->list_read_launch->addItem(select_launch_text_[i]);
		ui->list_read_launch->setCurrentRow(read_launch_select);
	}

	//保存されているステア校正用電圧値の取得
	{
		std::string param_path = ros::package::getPath("runtime_manager") + "/steer_voltage_param";
		std::ifstream ifs_param(param_path, std::ios_base::in);
		if(ifs_param)
		{
			std::string str_voltage_range;
			std::getline(ifs_param, str_voltage_range);
			rs232_steer_voltage_range_ = (uint16_t)atoi(str_voltage_range.c_str());

			std::string str_straight_point;
			std::getline(ifs_param, str_straight_point);
			rs232_steer_voltage_straight_ = (uint16_t)atoi(str_straight_point.c_str());

			std::cout << "steer_voltage," << rs232_steer_voltage_range_ << "," << rs232_steer_voltage_straight_ << std::endl;
		}
	}

	//保存されているrecord topic一覧を取得
	{
		std::string record_path = ros::package::getPath("runtime_manager") + "/record_topic";
		std::ifstream ifs_record(record_path, std::ios_base::in);
		if(ifs_record)
		{
			while(true)
			{
				std::string topic;
				std::getline(ifs_record, topic);
				std::cout << "topic," << topic << std::endl;
				record_topic_list_ += topic;
				if(ifs_record.eof()) break;
				else record_topic_list_ += ",";
			}
		}

		std_msgs::String str;
		str.data = record_topic_list_;
		pub_record_topic_list_.publish(str);
	}

	{
		waypoints_serial_num_.route_current = waypoints_serial_num_.route_next = "";
		waypoints_serial_num_.toAback = waypoints_serial_num_.toAnext = waypoints_serial_num_.toBback = waypoints_serial_num_.toBnext = "";
	}

	check_use_auto_chenge(ui->ch5_use_auto_change->isChecked());
	std_msgs::Bool msg_bool;
	msg_bool.data = ui->ch5_use_auto_log->isChecked();
	pub_auto_log_.publish(msg_bool);

	auto_route_loop_count_.lane = auto_route_loop_count_.lane = 0;

	ui->tx4_load_name->setAlignment(Qt::AlignCenter);

	move(1200, 0);

	int use_accel_intervention = nh_.param<int>("use_accel_intervention", 0);
	dialog_driving_adjustment_ = new Dialog_driving_adjustment(this);
	dialog_popup_signal_ = new Dialog_popup_signal((use_accel_intervention > 0) ? true : false, this);

	oncoming_obs_.existence = false;
}

MainWindow::~MainWindow()
{
	//log_folderのパスを保存
	{
		std::string log_folder_path = ros::package::getPath("runtime_manager") + "/param";
		std::ofstream ofs_log(log_folder_path, std::ios_base::out);
		if(ofs_log)
		{
			ofs_log << log_folder_ << '\n';
			ofs_log << ui->list_read_launch->currentRow() << '\n';
			ofs_log << ui->li2_steer_plus_interface->text().toStdString() << '\n';
			ofs_log << ui->sb_target_deceleration->value();
			ofs_log.close();
		}
	}

	delete dialog_driving_adjustment_;
    delete dialog_popup_signal_;

	system("rosnode kill /rviz_restart &");
	system("rosnode kill /auto_waypoints_change &");
	delete ui;
}

void MainWindow::error_view(std::string error_message)
{
	//std::cout << "abc : " << can_status_.safety_error_message << std::endl;
	std_msgs::Bool msg;
	msg.data = true;
	pub_error_lock_.publish(msg);

	ui->tx_error_text->setText(error_message.c_str());
	ui->tx2_error_text->setText(error_message.c_str());
	error_text_lock_ = true;
	if(can502_.clutch == true || can503_.clutch == true)
	{
		//system("aplay -D plughw:1,8 ~/one33.wav &");
		system("aplay ~/one33.wav &");
	}
	ros::Time now = ros::Time::now();
	timer_error_lock_ = ros::Time(now.sec+1, now.nsec);
}

void MainWindow::window_updata()
{
	const int keta = 3;
	bool unlock_flag = (can501_.emergency == false) ? true : false;

	ros::Time nowtime = ros::Time::now();

	ui->bt_drive_mode_manual->setEnabled(unlock_flag);
	ui->bt_drive_mode_program->setEnabled(unlock_flag);
	ui->bt_steer_mode_manual->setEnabled(unlock_flag);
	ui->bt_steer_mode_program->setEnabled(unlock_flag);
	ui->bt_blinker_left_on->setEnabled(unlock_flag);
	ui->bt_blinker_left_off->setEnabled(unlock_flag);
	ui->bt_blinker_right_on->setEnabled(unlock_flag);
	ui->bt_blinker_right_off->setEnabled(unlock_flag);
	ui->bt_drive_clutch_connect->setEnabled(unlock_flag);
	ui->bt_drive_clutch_cut->setEnabled(unlock_flag);
	ui->bt_steer_clutch_connect->setEnabled(unlock_flag);
	ui->bt_steer_clutch_cut->setEnabled(unlock_flag);

	if(unlock_flag)
	{
		//driveモードの状態
		std::stringstream str_drive_target;
		str_drive_target << can501_.stroke_reply;
		ui->tx_stroke_target->setText(str_drive_target.str().c_str());
		ui->tx2_drive_stroke_target->setText(str_drive_target.str().c_str());
		ui->tx5_drive_stroke_target->setText(str_drive_target.str().c_str());
		ui->bar_drive_cmd_value->setValue((int)can501_.stroke_reply);

		//double stroke = PEDAL_VOLTAGE_CENTER_ - can503_.pedal_voltage;
		double stroke = can503_.pedal_displacement;
		std::stringstream str_drive_actual;
		str_drive_actual << stroke;
		ui->tx_stroke_actual->setText(str_drive_actual.str().c_str());
		ui->tx2_drive_stroke_actual->setText(str_drive_actual.str().c_str());
		ui->tx5_drive_stroke_actual->setText(str_drive_actual.str().c_str());
		ui->bar_drive_joy_value->setValue((int)can503_.pedal_displacement);

		std::stringstream str_velocity;
		str_velocity << can502_.velocity_actual / 100;
		ui->tx_velocity_actual->setText(str_velocity.str().c_str());

		if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
		{
			ui->tx_drive_mode->setPalette(palette_drive_mode_ok_);
			ui->tx_drive_mode->setText("auto");
			ui->tx2_drive_mode->setPalette(palette_drive_mode_ok_);
			ui->tx2_drive_mode->setText("auto");
			ui->bt_drive_control_mode_velocity->setEnabled(true);
			ui->bt_drive_control_mode_stroke->setEnabled(true);
			ui->bt_drive_input_mode_direct->setEnabled(true);
			ui->bt_drive_input_mode_autoware->setEnabled(true);

			if(can501_.drive_mode == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY)
			{
				//ui->gb_velocity->setEnabled(true);
				//ui->gb_stroke->setEnabled(false);
				ui->tx_drive_control_mode->setText("velocity");
			}
			else if(can501_.drive_mode == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE)
			{
				//ui->gb_velocity->setEnabled(false);
				//ui->gb_stroke->setEnabled(true);
				ui->tx_drive_control_mode->setText("stroke");

				/*std::stringstream str_target;
				str_target << can501_.velocity;
				ui->tx_stroke_target->setText(str_target.str().c_str());

				double stroke = PEDAL_VOLTAGE_CENTER_ - can503_.pedal_voltage;
				std::stringstream str_actual;
				str_actual << stroke;
				ui->tx_stroke_actual->setText(str_actual.str().c_str());*/
			}
			else
			{
				//ui->gb_velocity->setEnabled(false);
				//ui->gb_stroke->setEnabled(false);
				ui->tx_drive_control_mode->setText("undefined");
			}
		}
		else
		{
			ui->tx_drive_control_mode->setText("none");
			ui->bt_drive_control_mode_velocity->setEnabled(false);
			ui->bt_drive_control_mode_stroke->setEnabled(false);
			ui->tx_drive_control_mode->setText("");
			ui->bt_drive_input_mode_direct->setEnabled(false);
			ui->bt_drive_input_mode_autoware->setEnabled(false);
			ui->tx_drive_input_mode->setText("");
			//ui->gb_velocity->setEnabled(false);
			//ui->gb_stroke->setEnabled(false);

			if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_V0)
			{
				ui->tx_drive_mode->setPalette(palette_drive_mode_ok_);
				ui->tx_drive_mode->setText("V0");
				ui->tx2_drive_mode->setPalette(palette_drive_mode_ok_);
				ui->tx2_drive_mode->setText("V0");
			}
			else
			{
				ui->tx_drive_mode->setPalette(palette_drive_mode_error_);
				ui->tx2_drive_mode->setPalette(palette_drive_mode_error_);
				if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_NOT_V0)
				{
					ui->tx_drive_mode->setText("1not V0");
					ui->tx2_drive_mode->setText("1not V0");
				}
				else if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_NOT_JOY_CENTER)
				{
					ui->tx_drive_mode->setText("2not joy center");
					ui->tx2_drive_mode->setText("2not joy center");
				}
				else if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_NOT_BOARD_RES)
				{
					ui->tx_drive_mode->setText("3not board res");
					ui->tx2_drive_mode->setText("3not board res");
				}
				else if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_V0 + autoware_can_msgs::MicroBusCan501::DRIVE_NOT_JOY_CENTER)
				{
					ui->tx_drive_mode->setText("4not V0\nnot joy center");
					ui->tx2_drive_mode->setText("4not V0\nnot joy center");
				}
				else if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_V0 + autoware_can_msgs::MicroBusCan501::DRIVE_NOT_BOARD_RES)
				{
					ui->tx_drive_mode->setText("5not V0\nnot board res");
					ui->tx2_drive_mode->setText("5not V0\nnot board res");
				}
				else if(can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_NOT_JOY_CENTER + autoware_can_msgs::MicroBusCan501::DRIVE_NOT_BOARD_RES)
				{
					ui->tx_drive_mode->setText("6not joy cente\nnot board res");
					ui->tx2_drive_mode->setText("6not joy cente\nnot board res");
				}
				else
				{
					ui->tx_drive_mode->setText("undefined");
					ui->tx2_drive_mode->setText("undefined");
				}
			}
		}

		{
			if(can502_.clutch == true && can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
			{
				ui->tx2_steer_auto_ok->setPalette(palette_steer_mode_ok_);
				ui->tx4_steer_auto_ok->setPalette(palette_steer_mode_ok_);
				ui->tx2_steer_auto_ok->setText("自動");
				ui->tx4_steer_auto_ok->setText("自動");
			}
			else
			{
				ui->tx2_steer_auto_ok->setPalette(palette_steer_mode_error_);
				ui->tx4_steer_auto_ok->setPalette(palette_steer_mode_error_);
				ui->tx2_steer_auto_ok->setText("手動");
				ui->tx4_steer_auto_ok->setText("手動");
			}
			if(can503_.clutch == true && can501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
			{
				ui->tx2_drive_auto_ok->setPalette(palette_drive_mode_ok_);
				ui->tx4_drive_auto_ok->setPalette(palette_drive_mode_ok_);
				ui->tx2_drive_auto_ok->setText("自動");
				ui->tx4_drive_auto_ok->setText("自動");
			}
			else
			{
				ui->tx2_drive_auto_ok->setPalette(palette_drive_mode_error_);
				ui->tx4_drive_auto_ok->setPalette(palette_drive_mode_error_);
				ui->tx2_drive_auto_ok->setText("手動");
				ui->tx4_drive_auto_ok->setText("手動");
			}
		}
		//steerモードの状態
		std::stringstream str_steer_target;
		str_steer_target << can501_.steering_angle_reply;
		ui->tx_angle_target->setText(str_steer_target.str().c_str());
		ui->tx2_steer_angle_target->setText(str_steer_target.str().c_str());
		ui->tx5_steer_target->setText(str_steer_target.str().c_str());
		ui->bar_steer_cmd_value->setValue((int)can501_.steering_angle_reply);

		std::stringstream str_steer_actual;
		str_steer_actual << can502_.angle_actual;
		ui->tx_angle_actual->setText(str_steer_actual.str().c_str());
		ui->tx2_steer_angle_actual->setText(str_steer_actual.str().c_str());
		ui->tx5_steer_actual->setText(str_steer_actual.str().c_str());
		ui->bar_steer_joy_value->setValue((int)can502_.angle_actual);

		if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
		{
			ui->tx_steer_mode->setPalette(palette_steer_mode_ok_);
			ui->tx_steer_mode->setText("auto");
			ui->tx2_steer_mode->setPalette(palette_steer_mode_ok_);
			ui->tx2_steer_mode->setText("auto");
			ui->bt_steer_input_mode_direct->setEnabled(true);
			ui->bt_steer_input_mode_autoware->setEnabled(true);
			//ui->gb_angle->setEnabled(true);

			/*std::stringstream str_target;
			str_target << can501_.steering_angle;
			ui->tx_angle_target->setText(str_target.str().c_str());

			std::stringstream str_actual;
			str_actual << can502_.angle_actual;
			ui->tx_angle_actual->setText(str_actual.str().c_str());*/
		}
		else
		{
			ui->bt_steer_input_mode_direct->setEnabled(false);
			ui->bt_steer_input_mode_autoware->setEnabled(false);
			ui->tx_steer_input_mode->setText("");
			//ui->gb_angle->setEnabled(false);

			if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_V0)
			{
				ui->tx_steer_mode->setPalette(palette_steer_mode_ok_);
				ui->tx_steer_mode->setText("V0");
				ui->tx2_steer_mode->setPalette(palette_steer_mode_ok_);
				ui->tx2_steer_mode->setText("V0");
			}
			else
			{
				ui->tx_steer_mode->setPalette(palette_steer_mode_error_);
				ui->tx2_steer_mode->setPalette(palette_steer_mode_error_);
				if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_NOT_V0)
				{
					ui->tx_steer_mode->setText("1not V0");
					ui->tx2_steer_mode->setText("1not V0");
				}
				else if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_NOT_JOY_CENTER)
				{
					ui->tx_steer_mode->setText("2not joy center");
					ui->tx2_steer_mode->setText("2not joy center");
				}
				else if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_NOT_BOARD_RES)
				{
					ui->tx_steer_mode->setText("3not board res");
					ui->tx2_steer_mode->setText("3not board res");
				}
				else if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_V0 + autoware_can_msgs::MicroBusCan501::STEER_NOT_JOY_CENTER)
				{
					ui->tx_steer_mode->setText("4not V0\nnot joy center");
					ui->tx2_steer_mode->setText("4not V0\nnot joy center");
				}
				else if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_V0 + autoware_can_msgs::MicroBusCan501::STEER_NOT_BOARD_RES)
				{
					ui->tx_steer_mode->setText("5not V0\nnot board res");
					ui->tx2_steer_mode->setText("5not V0\nnot board res");
				}
				else if(can501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_NOT_JOY_CENTER + autoware_can_msgs::MicroBusCan501::STEER_NOT_BOARD_RES)
				{
					ui->tx_steer_mode->setText("6not joy cente\nnot board res");
					ui->tx2_steer_mode->setText("6not joy cente\nnot board res");
				}
				else
				{
					ui->tx_steer_mode->setText("undefined");
					ui->tx2_steer_mode->setText("undefined");
				}
			}
		}

		//clutch
		if(can503_.clutch == true)
		{
			ui->tx_drive_clutch->setPalette(palette_drive_clutch_connect_);
			ui->tx2_drive_clutch->setPalette(palette_drive_clutch_connect_);
			ui->tx_drive_clutch->setText("connect");
			ui->tx2_drive_clutch->setText("connect");
		}
		else
		{
			ui->tx_drive_clutch->setPalette(palette_drive_clutch_cut_);
			ui->tx2_drive_clutch->setPalette(palette_drive_clutch_cut_);
			ui->tx_drive_clutch->setText("cut");
			ui->tx2_drive_clutch->setText("cut");
		}
		if(can502_.clutch == true)
		{
			ui->tx_steer_clutch->setPalette(palette_steer_clutch_connect_);
			ui->tx2_steer_clutch->setPalette(palette_steer_clutch_connect_);
			ui->tx_steer_clutch->setText("connect");
			ui->tx2_steer_clutch->setText("connect");
		}
		else
		{
			ui->tx_steer_clutch->setPalette(palette_steer_clutch_cut_);
			ui->tx2_steer_clutch->setPalette(palette_steer_clutch_cut_);
			ui->tx_steer_clutch->setText("cut");
			ui->tx2_steer_clutch->setText("cut");
		}

		//if(can502_.OT == false) 
		//入力モード
		if(can_status_.use_input_steer == true)
			ui->tx_steer_input_mode->setText("input");
		else
			ui->tx_steer_input_mode->setText("auto");
		if(can_status_.use_input_drive == true)
			ui->tx_drive_input_mode->setText("input");
		else
			ui->tx_drive_input_mode->setText("auto");
	}
	else
	{
		ui->tx_drive_mode->setText("");
		ui->tx2_drive_mode->setText("");
		ui->tx_drive_control_mode->setText("");
		ui->tx_drive_input_mode->setText("");
		ui->tx_steer_mode->setText("");
		ui->tx2_steer_mode->setText("");
		ui->tx_steer_input_mode->setText("");
		ui->tx_velocity_target->setText("");
		ui->tx_velocity_actual->setText("");
		ui->tx_stroke_target->setText("");
		ui->tx2_drive_stroke_target->setText("");
		ui->tx5_drive_stroke_target->setText("");
		ui->bar_drive_cmd_value->setValue(0);
		ui->tx_stroke_actual->setText("");
		ui->tx2_drive_stroke_actual->setText("");
		ui->tx5_drive_stroke_actual->setText("");
		ui->bar_drive_joy_value->setValue(0);
		ui->tx_angle_target->setText("");
		ui->tx2_steer_angle_target->setText("");
		ui->tx5_steer_target->setText("");
		ui->bar_steer_cmd_value->setValue(0);
		ui->tx_angle_actual->setText("");
		ui->tx2_steer_angle_actual->setText("");
		ui->tx5_steer_actual->setText("");
		ui->bar_steer_joy_value->setValue(0);
	}

	if(can_status_.safety_error_message != "" && error_text_lock_ == false)
		error_view(can_status_.safety_error_message);
	else if(can502_.OT == true)
		error_view(std::string("error : 502 OT"));
	else if(can502_.COF == true)
		error_view(std::string("error : 502 COF"));
	else if(can502_.BRK == true)
		error_view(std::string("error : 502 BRK"));
	else if(can502_.MLD == true)
		error_view(std::string("error : 502 MLD"));
	else if(can502_.CLD == true)
		error_view(std::string("error : 502 CLD"));
	else if(can502_.PLD == true)
		error_view(std::string("error : 502 PLD"));
	else if(can502_.JLD == true)
		error_view(std::string("error : 502 JLD"));
	else if(can503_.OT == true)
		error_view(std::string("error : 503 OT"));
	else if(can503_.PRE == true)
		error_view(std::string("error : 503 PRE"));
	else if(can503_.COF == true)
		error_view(std::string("error : 503 COF"));
	else if(can503_.BRK == true)
		error_view(std::string("error : 503 BRK"));
	else if(can503_.MLD == true)
		error_view(std::string("error : 503 MLD"));
	else if(can503_.CLD == true)
		error_view(std::string("error : 503 CLD"));
	else if(can503_.PLD)
		error_view(std::string("error : 503 PLD"));
	else if(can503_.JLD == true)
		error_view(std::string("error : 503 JLD"));

	if(fabs(distance_angular_check_.baselink_distance) <= config_.check_distance_th)
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance OK," << config_.check_distance_th << "," << distance_angular_check_.baselink_distance;
		ui->tx_distance_check->setText(str.str().c_str());
		ui->tx_distance_check->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance NG," << config_.check_distance_th << "," << distance_angular_check_.baselink_distance;
		ui->tx_distance_check->setText(str.str().c_str());
		ui->tx_distance_check->setPalette(palette_distance_angular_error_);
	}

	if(fabs(distance_angular_check_ndt_.baselink_distance) <= config_.check_distance_th)
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance OK," << config_.check_distance_th << "," << distance_angular_check_ndt_.baselink_distance;
		ui->tx_ndt_distance_check->setText(str.str().c_str());
		ui->tx_ndt_distance_check->setPalette(palette_distance_angular_ok_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_ndt_.baselink_distance;
		ui->tx2_ndt_distance->setText(str2.str().c_str());
		if(localizer_select_ == 0 || localizer_select_ == 10)
			ui->tx2_ndt_distance->setPalette(palette_current_localizer_);
		else
			ui->tx2_ndt_distance->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance NG," << config_.check_distance_th << "," << distance_angular_check_ndt_.baselink_distance;
		ui->tx_ndt_distance_check->setText(str.str().c_str());
		ui->tx_ndt_distance_check->setPalette(palette_distance_angular_error_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_ndt_.baselink_distance;
		ui->tx2_ndt_distance->setText(str2.str().c_str());
		ui->tx2_ndt_distance->setPalette(palette_distance_angular_error_);
	}

	if(fabs(distance_angular_check_ekf_.baselink_distance) <= config_.check_distance_th)
	{
		/*std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance OK," << config_.check_distance_th << "," << distance_angular_check_ekf_.baselink_distance;
		ui->tx_ekf_distance_check->setText(str.str().c_str());
		ui->tx_ekf_distance_check->setPalette(palette_distance_angular_ok_);*/

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_ekf_.baselink_distance;
		ui->tx2_ekf_distance->setText(str2.str().c_str());
		ui->tx2_ekf_distance->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		/*std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance NG," << config_.check_distance_th << "," << distance_angular_check_ekf_.baselink_distance;
		ui->tx_ekf_distance_check->setText(str.str().c_str());
		ui->tx_ekf_distance_check->setPalette(palette_distance_angular_error_);*/

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_ekf_.baselink_distance;
		ui->tx2_ekf_distance->setText(str2.str().c_str());
		ui->tx2_ekf_distance->setPalette(palette_distance_angular_error_);
	}
	
	if(fabs(distance_angular_check_gnss_.baselink_distance) <= config_.check_distance_th)
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance OK," << config_.check_distance_th << "," << distance_angular_check_gnss_.baselink_distance;
		ui->tx_gnss_distance_check->setText(str.str().c_str());
		ui->tx_gnss_distance_check->setPalette(palette_distance_angular_ok_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_gnss_.baselink_distance;
		ui->tx2_gnss_distance->setText(str2.str().c_str());
		if(localizer_select_ == 1 || localizer_select_ == 11)
			ui->tx2_gnss_distance->setPalette(palette_current_localizer_);
		else
			ui->tx2_gnss_distance->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "distance NG," << config_.check_distance_th << "," << distance_angular_check_gnss_.baselink_distance;
		ui->tx_gnss_distance_check->setText(str.str().c_str());
		ui->tx_gnss_distance_check->setPalette(palette_distance_angular_error_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_gnss_.baselink_distance;
		ui->tx2_gnss_distance->setText(str2.str().c_str());
		ui->tx2_gnss_distance->setPalette(palette_distance_angular_error_);
	}

	{
		double dif = distance_angular_check_ndt_.baselink_distance - distance_angular_check_gnss_.baselink_distance;
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << dif;
		ui->tx2_localizer_difference->setText(str.str().c_str());
	}

	double angular_deg = distance_angular_check_.baselink_angular * 180.0 / M_PI;
	double angular_deg_ndt = distance_angular_check_ndt_.baselink_angular * 180.0 / M_PI;
	double angular_deg_ekf = distance_angular_check_ekf_.baselink_angular * 180.0 / M_PI;
	double angular_deg_gnss = distance_angular_check_gnss_.baselink_angular * 180.0 / M_PI;
	if(fabs(angular_deg) <= config_.check_angular_th)
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular OK," << config_.check_angular_th << "," << angular_deg;
		ui->tx_angular_check->setText(str.str().c_str());
		ui->tx_angular_check->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular NG," << config_.check_angular_th << "," << angular_deg;
		ui->tx_angular_check->setText(str.str().c_str());
		ui->tx_angular_check->setPalette(palette_distance_angular_error_);
	}

	if(fabs(angular_deg_ndt) <= config_.check_angular_th)
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular OK," << config_.check_angular_th << "," << angular_deg_ndt;
		ui->tx_ndt_angular_check->setText(str.str().c_str());
		ui->tx_ndt_angular_check->setPalette(palette_distance_angular_ok_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_ndt_.baselink_angular;
		ui->tx2_ndt_angular->setText(str2.str().c_str());
		if(localizer_select_ == 0 || localizer_select_ == 10)
			ui->tx2_ndt_angular->setPalette(palette_current_localizer_);
		else
			ui->tx2_ndt_angular->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular NG," << config_.check_angular_th << "," << angular_deg_ndt;
		ui->tx_ndt_angular_check->setText(str.str().c_str());
		ui->tx_ndt_angular_check->setPalette(palette_distance_angular_error_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << angular_deg_ndt;
		ui->tx2_ndt_angular->setText(str2.str().c_str());
		ui->tx2_ndt_angular->setPalette(palette_distance_angular_error_);
	}

	if(fabs(angular_deg_ekf) <= config_.check_angular_th)
	{
		/*std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular OK," << config_.check_angular_th << "," << angular_deg_ekf;
		ui->tx_ekf_angular_check->setText(str.str().c_str());
		ui->tx_ekf_angular_check->setPalette(palette_distance_angular_ok_);*/

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << distance_angular_check_ekf_.baselink_angular;
		ui->tx2_ekf_angular->setText(str2.str().c_str());
		ui->tx2_ekf_angular->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		/*std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular NG," << config_.check_angular_th << "," << angular_deg_ekf;
		ui->tx_ekf_angular_check->setText(str.str().c_str());
		ui->tx_ekf_angular_check->setPalette(palette_distance_angular_error_);*/

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << angular_deg_ekf;
		ui->tx2_ekf_angular->setText(str2.str().c_str());
		ui->tx2_ekf_angular->setPalette(palette_distance_angular_error_);
	}

	if(fabs(angular_deg_gnss) <= config_.check_angular_th)
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular OK," << config_.check_angular_th << "," << angular_deg_gnss;
		ui->tx_gnss_angular_check->setText(str.str().c_str());
		ui->tx_gnss_angular_check->setPalette(palette_distance_angular_ok_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << angular_deg_gnss;
		ui->tx2_gnss_angular->setText(str2.str().c_str());
		if(localizer_select_ == 1 || localizer_select_ == 11)
			ui->tx2_gnss_angular->setPalette(palette_current_localizer_);
		else
			ui->tx2_gnss_angular->setPalette(palette_distance_angular_ok_);
	}
	else
	{
		std::stringstream str;
		str << std::fixed << std::setprecision(keta) << "angular NG," << config_.check_angular_th << "," << angular_deg_gnss;
		ui->tx_gnss_angular_check->setText(str.str().c_str());
		ui->tx_gnss_angular_check->setPalette(palette_distance_angular_error_);

		std::stringstream str2;
		str2 << std::fixed << std::setprecision(keta) << angular_deg_gnss;
		ui->tx2_gnss_angular->setText(str2.str().c_str());
		ui->tx2_gnss_angular->setPalette(palette_distance_angular_error_);
	}

	{
		std::stringstream str;
		switch(localizer_select_)
		{
			case 0:
				ui->tx_localizer_select->setText("NDT+ODOM");
				ui->tx_localizer_select->setPalette(palette_localizer_select_ok_);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(true);
				ui->lb2_ndt->setAutoFillBackground(false);
				break;
			case 10:
				ui->tx_localizer_select->setText("GNSS+GYLO->NDT+ODOM");
				ui->tx_localizer_select->setPalette(palette_localizer_select_ok_);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(true);
				ui->lb2_ndt->setAutoFillBackground(false);
				break;
			case 1:
				ui->tx_localizer_select->setText("GNSS+GYLO");
				ui->tx_localizer_select->setPalette(palette_localizer_select_ok_);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(true);
				break;
			case 11:
				ui->tx_localizer_select->setText("NDT+ODOM->GNSS+GYLO");
				ui->tx_localizer_select->setPalette(palette_localizer_select_ok_);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(true);
				break;
			default:
				ui->tx_localizer_select->setText("distance too large");
				ui->tx_localizer_select->setPalette(palette_localizer_select_error_);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(false);
				ui->lb2_ndt->setAutoFillBackground(false);
		}
		
	}

	{
		std::stringstream str;
		std::string stat = (localizer_match_stat_.localizer_stat == true) ? "true" : "false";
		str << stat << "," << localizer_match_stat_.localizer_distance;
		ui->tx_localizer_match_stat->setText(str.str().c_str());
	}

	{
		std::stringstream str_vel;
		str_vel << std::fixed << std::setprecision(keta) << current_velocity_.twist.linear.x * 3.6;//can_velocity_param_.velocity * 3.6;
		ui->tx_can_velocity->setText(str_vel.str().c_str());
		ui->tx2_cur_vel->setText(str_vel.str().c_str());
		ui->tx4_cur_vel->setText(str_vel.str().c_str());

		std::stringstream str_vehicle_cmd;
		str_vehicle_cmd << std::fixed << std::setprecision(keta) << vehicle_cmd_.ctrl_cmd.linear_velocity * 3.6;
		ui->tx2_cmd_vel->setText(str_vehicle_cmd.str().c_str());

		std::stringstream str_way_vel;
		str_way_vel << std::fixed << std::setprecision(keta) << waypoint_param_.global_twist.linear.x * 3.6;
		ui->tx2_way_vel->setText(str_way_vel.str().c_str());

		std::stringstream str_jurk;
		str_jurk << std::fixed << std::setprecision(keta) << can_velocity_param_.jurk;
		ui->tx_can_jurk->setText(str_jurk.str().c_str());
		ui->tx2_jurk->setText(str_jurk.str().c_str());

		std::stringstream str_stop_dis;
		str_stop_dis  << std::fixed << std::setprecision(keta) << stopper_distance_.distance;
		ui->tx_stopper_distance->setText(str_stop_dis.str().c_str());
		ui->tx2_stopD->setText(str_stop_dis.str().c_str());

		std::stringstream str_way_num;
		str_way_num << waypoint_param_.id;
		ui->tx_waypoint_num->setText(str_way_num.str().c_str());
		ui->tx2_waypoint_num->setText(str_way_num.str().c_str());

		std::stringstream str_acc;
		str_acc << std::fixed << std::setprecision(keta) << acc_ / 9.8;
		ui->tx2_acc->setText(str_acc.str().c_str());
		ui->tx4_acc->setText(str_acc.str().c_str());
	}

	{
		std::stringstream str_lat, str_lon, str_alt, str_yaw_dev;
		str_lat << std::fixed << std::setprecision(keta) << gnss_deviation_.lat_std_dev;
		if(gnss_deviation_.lat_std_dev > config_.gnss_lat_limit)
		{
			ui->tx_lat->setPalette(palette_gnss_deviation_error_);
			ui->tx2_gnss_lat->setPalette(palette_gnss_deviation_error_);
		}
		else
		{
			ui->tx_lat->setPalette(palette_gnss_deviation_ok_);
			ui->tx2_gnss_lat->setPalette(palette_gnss_deviation_ok_);
		}
		ui->tx_lat->setText(str_lat.str().c_str());
		ui->tx2_gnss_lat->setText(str_lat.str().c_str());

		str_lon << std::fixed << std::setprecision(keta) << gnss_deviation_.lon_std_dev;
		if(gnss_deviation_.lon_std_dev > config_.gnss_lon_limit)
		{
			ui->tx_lon->setPalette(palette_gnss_deviation_error_);
			ui->tx2_gnss_lon->setPalette(palette_gnss_deviation_error_);
		}
		else
		{
			ui->tx_lon->setPalette(palette_gnss_deviation_ok_);
			ui->tx2_gnss_lon->setPalette(palette_gnss_deviation_ok_);
		}
		ui->tx_lon->setText(str_lon.str().c_str());
		ui->tx2_gnss_lon->setText(str_lon.str().c_str());

		str_alt << std::fixed << std::setprecision(keta) << gnss_deviation_.alt_std_dev;
		if(gnss_deviation_.alt_std_dev > config_.gnss_alt_limit)
		{
			ui->tx_alt->setPalette(palette_gnss_deviation_error_);
			ui->tx2_gnss_alt->setPalette(palette_gnss_deviation_error_);
		}
		else
		{
			ui->tx_alt->setPalette(palette_gnss_deviation_ok_);
			ui->tx2_gnss_alt->setPalette(palette_gnss_deviation_ok_);
		}
		ui->tx_alt->setText(str_alt.str().c_str());
		ui->tx2_gnss_alt->setText(str_alt.str().c_str());

		str_yaw_dev << std::fixed << std::setprecision(keta) << gnss_deviation_.azimuth_std_dev*180/M_PI;
		if(gnss_deviation_.azimuth_std_dev > config_.gnss_yaw_limit)
		{
			ui->tx_yaw_dev->setPalette(palette_gnss_deviation_error_);
			ui->tx2_gnss_yaw_dev->setPalette(palette_gnss_deviation_error_);
		}
		else
		{
			ui->tx_yaw_dev->setPalette(palette_gnss_deviation_ok_);
			ui->tx2_gnss_yaw_dev->setPalette(palette_gnss_deviation_ok_);
		}
		ui->tx_yaw_dev->setText(str_yaw_dev.str().c_str());
		ui->tx2_gnss_yaw_dev->setText(str_yaw_dev.str().c_str());
	}

	{
		std::stringstream str_ndt_stat, str_ndt_score, str_ndt_time, str_leaf_size, str_range;

		str_ndt_stat << std::fixed << std::setprecision(keta) << ndt_stat_.score;
		ui->tx_ndt_score->setText(str_ndt_stat.str().c_str());
		ui->tx2_ndt_score->setText(str_ndt_stat.str().c_str());

		str_ndt_time << std::fixed << std::setprecision(keta) << ndt_stat_.exe_time;
		ui->tx2_ndt_time->setText(str_ndt_time.str().c_str());

		str_leaf_size << std::fixed << std::setprecision(keta) << config_voxel_grid_filter_.voxel_leaf_size;
		ui->tx2_leaf_size->setText(str_leaf_size.str().c_str());

		str_range << std::fixed << std::setprecision(keta) << config_voxel_grid_filter_.measurement_range;
		ui->tx2_ndt_range->setText(str_range.str().c_str());

		if(gnss_stat_ == 3)
		{
			ui->tx_gnss_ok->setPalette(palette_gnss_deviation_ok_);
			ui->tx_gnss_ok->setText("OK");
			ui->tx2_gnss_ok->setPalette(palette_gnss_deviation_ok_);
			ui->tx2_gnss_ok->setText("OK");
		}
		else
		{
			ui->tx_gnss_ok->setPalette(palette_gnss_deviation_error_);
			ui->tx_gnss_ok->setText("NG");
			ui->tx2_gnss_ok->setPalette(palette_gnss_deviation_error_);
			ui->tx2_gnss_ok->setText("NG");
		}

		if(ndt_stat_string_ == "NDT_OK")
		{
			ui->tx_ndt_ok->setPalette(palette_gnss_deviation_ok_);
			ui->tx_ndt_ok->setText("OK");
			ui->tx2_ndt_ok->setPalette(palette_gnss_deviation_ok_);
			ui->tx2_ndt_ok->setText("OK");
		}
		else
		{
			ui->tx_ndt_ok->setPalette(palette_gnss_deviation_error_);
			ui->tx_ndt_ok->setText("NG");
			ui->tx2_ndt_ok->setPalette(palette_gnss_deviation_error_);
			ui->tx2_ndt_ok->setText("NG");
		}
	}

	{
		ui->tx_stroke_routine->setText(stroke_routine_.c_str());
		ui->tx2_stroke_routine->setText(stroke_routine_.c_str());
	}

	{
		double yaw, roll, pitch;
		tf::Quaternion qua;
		tf::quaternionMsgToTF(gnss_pose_.pose.orientation, qua);
		tf::Matrix3x3 mat(qua);
		mat.getRPY(roll, pitch, yaw);

		std::stringstream str_yaw, str_roll, str_pitch;
		str_yaw << std::setprecision(keta) << yaw*180/M_PI;
		ui->tx_yaw->setText(str_yaw.str().c_str());
		ui->tx2_gnss_yaw->setText(str_yaw.str().c_str());
		str_roll << std::setprecision(keta) << roll*180/M_PI;
		ui->tx_roll->setText(str_roll.str().c_str());
		ui->tx2_gnss_roll->setText(str_roll.str().c_str());
		str_pitch << std::setprecision(keta) << pitch*180/M_PI;
		ui->tx_pitch->setText(str_pitch.str().c_str());
		ui->tx2_gnss_pitch->setText(str_pitch.str().c_str());
	}

	{
		if(mobileye_lane_.lane_type_left != mobileye_560_660_msgs::AftermarketLane::LANE_TYPE_NONE &&
		   mobileye_lane_.lane_confidence_left >= 1)
		{
			std::stringstream str_left;
			str_left << std::fixed << std::setprecision(keta) << mobileye_lane_.distance_to_left_lane;
			ui->tx2_left_lane->setText(str_left.str().c_str());
		}
		else ui->tx2_left_lane->setText("NONE");

		if(mobileye_lane_.lane_type_right != mobileye_560_660_msgs::AftermarketLane::LANE_TYPE_NONE &&
		   mobileye_lane_.lane_confidence_right >= 1)
		{
			std::stringstream str_right;
			str_right << std::fixed << std::setprecision(keta) << mobileye_lane_.distance_to_right_lane;
			ui->tx2_right_lane->setText(str_right.str().c_str());
		}
		else ui->tx2_right_lane->setText("NONE");
	}

	{
		ui->tx2_log_folder->setText(log_folder_.c_str());
	}

	{
		switch(light_color_.traffic_light)
		{
		case MainWindow::TRAFFIC_LIGHT_GREEN:
			ui->tx2_signal_color->setText("GREEN");
			ui->tx2_signal_color->setPalette(palette_signal_text_green_);
			break;
		case MainWindow::TRAFFIC_LIGHT_RED:
			ui->tx2_signal_color->setText("RED");
			ui->tx2_signal_color->setPalette(palette_signal_text_red_);
			break;
		default:
			ui->tx2_signal_color->setText("UNKNOWN");
			ui->tx2_signal_color->setPalette(palette_signal_text_unknown_);
			break;
		}

		if(light_color_.traffic_light == MainWindow::TRAFFIC_LIGHT_UNKNOWN)
		{
			std::stringstream str_change_time;
			str_change_time << std::fixed << std::setprecision(keta) << signal_change_time_;
			ui->tx2_signal_change_time->setText(str_change_time.str().c_str());

			std::string str_period_signal_takeover = (period_signal_takeover_) ? "It'll soon turn RED" : ""; //青信号だけど、そろそろ赤に変わりそうな場合
			ui->tx2_signal_change_time->setText(str_period_signal_takeover.c_str());
		}
		else
		{
			ui->tx2_signal_change_time->setText("");
			ui->tx2_period_signal_takeover->setText("");
		}

		std::stringstream str_mileage;
		str_mileage << std::fixed << std::setprecision(keta) << automode_mileage_;
		ui->tx_automode_mileage->setText(str_mileage.str().c_str());

		std::stringstream str_pedal_voltage_center;
		str_pedal_voltage_center << config_.pedal_center_voltage;
		ui->tx2_pedal_vol_center->setText(str_pedal_voltage_center.str().c_str());

		std::stringstream str_pedal_voltage_diff;
		str_pedal_voltage_diff << config_.pedal_center_voltage - can503_.pedal_voltage;
		ui->tx2_pedal_vol_diff->setText(str_pedal_voltage_diff.str().c_str());
	}

	{
		//if(can502_.auto_mode == false && can503_.auto_mode == true) ui->cb_use_localizer_safety->setChecked(false);
		//else ui->cb_use_localizer_safety->setChecked(true);
	}

	{
		if(stopper_distance_.fixed_velocity == 0 &&
		   stopper_distance_.distance > -1)
		{
			if(stopper_distance_.distance <= 0.5 && can_velocity_param_.velocity < 0.04)
				ui->tx2_stopD->setPalette(palette_stop_line_stop_);
			else ui->tx2_stopD->setPalette(palette_stop_line_middle_);
		}
		else ui->tx2_stopD->setPalette(palette_stop_line_non_);
	}

	{
		std::stringstream str_cmd_select;
		switch (cmd_select_)
		{
		case 1:
			str_cmd_select << "MPC";
			break;
		case 2:
			str_cmd_select << "PURE";
			break;
		default:
			str_cmd_select << "UNKNOW";
		}
		ui->tx2_cmd_node->setText(str_cmd_select.str().c_str());
	}

	{
		if(sub_base_waypoints_.getNumPublishers() > 0)
		{
			ui->tx4_read_global_waypoints->setText("OK");
		}
		else
		{
			ui->tx4_read_global_waypoints->setText("NG");
		}

		//ros::Duration localizer_time_diff = nowtime - current_velocity_.header.stamp;
		//double localizer_time_dt = localizer_time_diff.sec + localizer_time_diff.nsec * 1E-9;
		double localizer_time_dt = localizer_time_diff_.sec + localizer_time_diff_.nsec * 1E-9;

		int ok_flag = 0;
		if(gnss_deviation_.lat_std_dev >= config_.gnss_lat_limit) ok_flag = 1;
		if(gnss_deviation_.lon_std_dev >= config_.gnss_lon_limit) ok_flag = 2;
		if(gnss_deviation_.alt_std_dev >= config_.gnss_alt_limit) ok_flag = 3;
		if(fabs(distance_angular_check_.baselink_distance) >= config_.check_distance_th) ok_flag = 4;
		if(fabs(distance_angular_check_.baselink_angular) >= config_.check_angular_th) ok_flag = 5;
		if(gnss_stat_ != 3) ok_flag = 6;
		if(localizer_time_dt >= 0.4) ok_flag = 7;
		if(read_safety_waypoints_ == false) ok_flag = 8;

		if(ok_flag == 0)
		{
			ui->tx2_auto_ok->setText("○");
			ui->tx2_auto_ok->setPalette(palette_auto_check_ok_);
			ui->tx4_auto_ok->setText("○");
			ui->tx4_auto_ok->setPalette(palette_auto_check_ok_);
		}
		else
		{
			//std::string str = std::to_string(ok_flag);
			ui->tx2_auto_ok->setText("×");
			ui->tx2_auto_ok->setPalette(palette_auto_check_error_);
			ui->tx4_auto_ok->setText("×");
			ui->tx4_auto_ok->setPalette(palette_auto_check_error_);
		}
		
		/*if(gnss_deviation_.lat_std_dev < config_.gnss_lat_limit &&
		   gnss_deviation_.lon_std_dev < config_.gnss_lon_limit &&
		   gnss_deviation_.alt_std_dev < config_.gnss_alt_limit &&
		   fabs(distance_angular_check_.baselink_distance) < config_.check_distance_th &&
		   fabs(distance_angular_check_.baselink_angular) < config_.check_angular_th &&
		   gnss_stat_ == 3 &&
		   localizer_time_dt < 0.4 &&
		   read_safety_waypoints_ == true)
		{
			ui->tx2_auto_ok->setText("○");
			ui->tx2_auto_ok->setPalette(palette_auto_check_ok_);
			ui->tx4_auto_ok->setText("○");
			ui->tx4_auto_ok->setPalette(palette_auto_check_ok_);
		}
		else
		{
			ui->tx2_auto_ok->setText("×");
			ui->tx2_auto_ok->setPalette(palette_auto_check_error_);
			ui->tx4_auto_ok->setText("×");
			ui->tx4_auto_ok->setPalette(palette_auto_check_error_);
		}*/
	}

	{
		if(use_specified_cmd_ == true)
		{
			std::stringstream ss_vel;
			ss_vel << can502_.velocity_mps * 3.6;
			ui->li5_can_velocity->setText(ss_vel.str().c_str());

			std::stringstream ss_steer;
			if(can502_.angle_actual > 0) ss_steer << (can502_.angle_actual / wheelrad_to_steering_can_value_left_) * 180 / M_PI;
			else ss_steer << (can502_.angle_actual / wheelrad_to_steering_can_value_right_) * 180 / M_PI;
			ui->li5_can_deg->setText(ss_steer.str().c_str());

			char *str_speed = ui->li5_can_send_velocity_->text().toUtf8().data();
			double speed = atof(str_speed);
			speed /= 3.6;

			char *str_deg = ui->li5_can_send_deg->text().toUtf8().data();
			double deg = atof(str_deg);
			double rad = deg * M_PI / 180.0;

			autoware_msgs::VehicleCmd cmd;
			cmd.header.stamp = ros::Time(0);
			cmd.ctrl_cmd.linear_velocity = speed;
			cmd.ctrl_cmd.linear_acceleration = cmd.ctrl_cmd.linear_acceleration = 0;
			cmd.ctrl_cmd.steering_angle = rad;
			pub_vehicle_cmd_.publish(cmd);
		}
	}

	if(use_steer_plus_sum_)
	{
		/*steer_plus_sum_ += can502_.angle_actual - config_.steer_actual_plus;
		steer_plus_sum_count_++;
		int val = (int)(steer_plus_sum_ / steer_plus_sum_count_);
		char buf[10];
		sprintf(buf, "%d", -val);
		ui->li2_steer_plus_interface->setText(buf);*/
	}

	//steer 調整関連
	{
		ui->li2_steer_plus_param_->setText(std::to_string(waypoint_param_.steer_actual_plus).c_str());
		ui->li2_steer_plus_param_sub->setText(std::to_string(waypoint_param_.steer_actual_plus_sub).c_str());

		autoware_msgs::SteerSubInterfaceCorrection msg;
		msg.header.stamp = nowtime;
		msg.processing = (dialog_driving_adjustment_->isHidden()) ? false : true;
		msg.correct_val = dialog_driving_adjustment_->getSubSteer();
		pub_steer_sub_interface_correction_.publish(msg);
	}

	{
		std::stringstream ss;
		ss << std::fixed << std::setprecision(1) << config_.velocity_limit << std::endl;
		ui->li_max_speed->setText(ss.str().c_str());
	}

	//パネル表示
	{
		uint8_t front_signal = panel_flag_front_.getViewSignal();
		uint8_t back_signal = panel_flag_back_.getViewSignal();

		uint16_t signal_msg = (((uint16_t)front_signal) << 8) + (uint8_t)back_signal;
		if(panel_signal_msg_ != signal_msg)
		{
			if(front_signal == autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO || back_signal == autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO)
			{
				std_msgs::Empty reset_msg;
				pub_panel_view_reset_.publish(reset_msg);
			}
			else
			{
				std_msgs::UInt16 msg;
				msg.data = signal_msg;
				pub_panel_view_.publish(msg);
			}
			panel_signal_msg_ = signal_msg;
		}

		if(panel_view_front_ > 0 && panel_view_front_ <= panel_front_text_.size())
			ui->tx4_panel_view_front->setText(panel_front_text_[panel_view_front_-1]);
		else
			ui->tx4_panel_view_front->setText("");
		if(panel_view_back_ > 0 && panel_view_back_ <= panel_back_text_.size())
			ui->tx4_panel_view_back->setText(panel_back_text_[panel_view_back_-1]);
		else
			ui->tx4_panel_view_back->setText("");
	}

	//mobileye
	{
		std::stringstream ss1, ss2, ss3;
		ss1 << std::fixed << std::setprecision(2) << mobileye_front_car_.velocity_mps * 3.6;
		ui->li_front_car_velocity->setText(ss1.str().c_str());
		ui->li_tracking_type->setText(tracking_type_.c_str());

		ss2 << std::fixed << std::setprecision(2) << mobileye_cmd_param_.L0;
		ui->li_front_car_distance->setText(ss2.str().c_str());

		ss3 << std::fixed << std::setprecision(2) << mobileye_cmd_param_.Lt_RSS;
		ui->li_front_car_responsibility_distance->setText(ss3.str().c_str());
	}
}

void MainWindow::callbackConfig(const autoware_config_msgs::ConfigMicroBusCan &msg)
{
	config_ = msg;
}

void MainWindow::callbackLocalizerSelect(const std_msgs::Int32 &msg)
{
	localizer_select_ = msg.data;
}

void MainWindow::callbackLocalizerMatchStat(const autoware_msgs::LocalizerMatchStat &msg)
{
	localizer_match_stat_ = msg;
}

void MainWindow::dialog_driving_adjustment_show()
{
	dialog_driving_adjustment_->show();//ステア調整フォーム起動
	system("rosrun microbus_interface steer_correction_write &");//ステア調整書き込みノード起動
}

void MainWindow::callbackCan501(const autoware_can_msgs::MicroBusCan501 &msg)
{
	//走行調整機能使用時
	if(ui->cb4_dialog_driving_adjustment->isChecked())
	{
		//std::cout << "din_in," << can501_.din0 << "," << can501_.din1 << ","  << can501_.din2 << "," << can501_.din3 << std::endl;
		dialog_driving_adjustment_->setDin(can501_.din0, can501_.din1, can501_.din2, can501_.din3);
	}
	else
	{
		dialog_driving_adjustment_->setDin(false, false, false, false);

        if(dialog_driving_adjustment_->isHidden() == true && dialog_popup_signal_->isHidden() == false)// && can502_.clutch == true)
		{
			//dialog_driving_adjustment_->setDin(false, false, false, false);
            		dialog_popup_signal_->setDin(can501_.din0, can501_.din1, can501_.din2, can501_.din3, can501_.accel_intervention);
		}
		else
		{
            		dialog_popup_signal_->setDin(false, false, false, false, false);
		}

        if(dialog_driving_adjustment_->isHidden() == true && dialog_popup_signal_->getProcessFlag() == false)
		{
			//速度調整
			std_msgs::Float64 speed;
			if(msg.din1 == true)//速度上昇
			{
				config_.velocity_limit += 0.1;
				if(config_.velocity_limit > 100) config_.velocity_limit = 100;
				speed.data = config_.velocity_limit;
				pub_change_max_speed_.publish(speed);
			}
			else if(msg.din0 == true)//速度下降
			{
				config_.velocity_limit -= 0.1;
				if(config_.velocity_limit < 0) config_.velocity_limit = 0;
				speed.data = config_.velocity_limit;
				pub_change_max_speed_.publish(speed);
			}
		}
	}

	if(msg.auto_log == true) rosbag_write();
	else rosbag_stop();
	can501_ = msg;
}

void MainWindow::callbackCan502(const autoware_can_msgs::MicroBusCan502 &msg)
{
	ros::Duration time_diff = timer_error_lock_ - ros::Time::now();
	if(ui->cb_use_clutch->isChecked() == true && msg.clutch == true && error_text_lock_ == true && time_diff < ros::Duration(0))
		click_error_text_reset();
	can502_ = msg;
}

void MainWindow::callbackCan503(const autoware_can_msgs::MicroBusCan503 &msg)
{
	ros::Duration time_diff = timer_error_lock_ - ros::Time::now();
	if(ui->cb_use_clutch->isChecked() == true && msg.clutch == true && error_text_lock_ == true && time_diff < ros::Duration(0))
		click_error_text_reset();
	can503_ = msg;
}

void MainWindow::callbackCanStatus(const autoware_can_msgs::MicroBusCanSenderStatus &msg)
{
	can_status_ = msg;
}

void MainWindow::callbackDistanceAngularCheck(const autoware_msgs::DifferenceToWaypointDistance &msg)
{
	distance_angular_check_ = msg;
}

void MainWindow::callbackDistanceAngularCheckNdt(const autoware_msgs::DifferenceToWaypointDistance &msg)
{
	distance_angular_check_ndt_ = msg;
}

void MainWindow::callbackDistanceAngularCheckEkf(const autoware_msgs::DifferenceToWaypointDistance &msg)
{
	distance_angular_check_ekf_ = msg;
}

void MainWindow::callbackDistanceAngularCheckGnss(const autoware_msgs::DifferenceToWaypointDistance &msg)
{
	distance_angular_check_gnss_ = msg;
}

void MainWindow::callbackImu(const sensor_msgs::Imu & msg)
{
	imu_ = msg;
}

void MainWindow::callbackCanVelocityParam(const autoware_can_msgs::MicroBusCanVelocityParam &msg)
{
	can_velocity_param_ = msg;
}

void MainWindow::callbackStopperDistance(const autoware_msgs::StopperDistance &msg)
{
	stopper_distance_ = msg;
}

void MainWindow::callbackWaypointParam(const autoware_msgs::WaypointParam &msg)
{
	dialog_driving_adjustment_->setWaypointCorrection(msg.steer_actual_plus, msg.steer_actual_plus_sub);

	/*if(can502_.clutch == true)
	{
		switch(msg.blinker)
		{
		case 1://左
			if(msg.blinker != waypoint_param_.blinker)
			{
                dialog_popup_signal_->setArrow(1);
                dialog_popup_signal_->show();
			}
			break;
		case 2://右
			if(msg.blinker != waypoint_param_.blinker)
			{
                dialog_popup_signal_->setArrow(2);
                dialog_popup_signal_->show();
			}
			break;
		case 0://停止
			if(msg.blinker != waypoint_param_.blinker)
			{
                int arrow = dialog_popup_signal_->getArrow();
				if(arrow == 1 || arrow == 2)
					publish_blinker_stop();
                dialog_popup_signal_->setArrow(-1);
                dialog_popup_signal_->close();
			}
			break;
		}
	}*/

	waypoint_param_ = msg;
}

void MainWindow::callbackNdtStat(const autoware_msgs::NDTStat &msg)
{
	ndt_stat_ = msg;
}

void MainWindow::callbackGnssPose(const geometry_msgs::PoseStamped &msg)
{
	gnss_pose_ = msg;
}

void MainWindow::callbackGnssDeviation(const autoware_msgs::GnssStandardDeviation &msg)
{
	gnss_deviation_ = msg;
}

void MainWindow::callbackGnssStat(const std_msgs::UInt8 &msg)
{
	gnss_stat_ = msg.data;
}

void MainWindow::callbackNdtStatString(const std_msgs::String &msg)
{
	ndt_stat_string_ = msg.data;
}

void MainWindow::callbackStrokeRoutine(const std_msgs::String &msg)
{
	stroke_routine_ = msg.data;
}

void MainWindow::callbackLightColor(const autoware_msgs::TrafficLight &msg)
{
	light_color_ = msg;
}

void MainWindow::callbackSignalChangeTime(const std_msgs::Float64 &msg)
{
	signal_change_time_ = msg.data;
}

void MainWindow::callbackPeriodSignalTakeover(const std_msgs::Bool &msg)
{
	period_signal_takeover_ = msg.data;
}

const bool getMessage_bool(const unsigned char *buf, unsigned int bit)
{
	unsigned long long mask=1;
	mask<<=bit;
	unsigned long long *msgL=(unsigned long long)buf;
	if((*msgL & mask)) return true;
	else return false;
}

template<typename T>
const T getMessage_bit(const unsigned char *buf, const unsigned int lowBit, const unsigned int highBit)
{
	const unsigned int maxBitSize=sizeof(unsigned long long)*8;
	unsigned long long *msgL=(unsigned long long)buf;
	unsigned long long val=(*msgL)<<maxBitSize-highBit-1;
	unsigned int lowPos=lowBit+(maxBitSize-highBit-1);
	val>>=lowPos;
	return (T)val;
}

void MainWindow::callbackMobileyeCan(const can_msgs::Frame &frame)
{
	switch(frame.id)
	{
	case 0x669:
		{
			if(frame.is_error == false && frame.dlc == 8)
			{
				const unsigned char *buf = (unsigned char*)frame.data.data();
				//Lane type
				mobileye_lane_.lane_type_left = getMessage_bit<unsigned char>(&buf[0], 4, 7);
				mobileye_lane_.lane_type_right = getMessage_bit<unsigned char>(&buf[5], 4, 7);
				//ldw_available
				mobileye_lane_.ldw_available_left = getMessage_bool(&buf[0], 2);
				mobileye_lane_.ldw_available_right = getMessage_bool(&buf[5], 2);
				//lane_confidence
				mobileye_lane_.lane_confidence_left = getMessage_bit<unsigned char>(&buf[0], 0, 1);
				mobileye_lane_.lane_confidence_right = getMessage_bit<unsigned char>(&buf[5], 0, 1);
				//distance_to lane
				int16_t distL, distR;
				unsigned char* distL_p = (unsigned char*)&distL;
				distL_p[1] = getMessage_bit<unsigned char>(&buf[2], 4, 7);
				distL_p[0] = getMessage_bit<unsigned char>(&buf[2], 0, 3) << 4;
				distL_p[0] |= getMessage_bit<unsigned char>(&buf[1], 4, 7);
				if(distL_p[1] & 0x8)//12bitのマイナスか
				{
					distL--;
					distL = ~distL;
					distL_p[1] &= 0x0F;
					distL = -distL;
				}
				mobileye_lane_.distance_to_left_lane = distL * 0.02;
				std::cout << "distL : " << (int)distL << std::endl;
				unsigned char* distR_p = (unsigned char*)&distR;
				distR_p[1] = getMessage_bit<unsigned char>(&buf[7], 4, 7);
				distR_p[0] = getMessage_bit<unsigned char>(&buf[7], 0, 3) << 4;
				distR_p[0] |= getMessage_bit<unsigned char>(&buf[6], 4, 7);
				if(distR_p[1] & 0x8)//12bitのマイナス化
				{
					distR--;
					distR = ~distR;
					distR_p[1] &= 0x0F;
					distR = -distR;
				}
				mobileye_lane_.distance_to_right_lane = distR * 0.02;
				std::cout << "distR : " << (int)distR << std::endl;
			}
			break;
		}
	}
}

void MainWindow::callbackOncomingObs(const autoware_msgs::NearOncomingObs::ConstPtr &msg)
{
	oncoming_obs_ = *msg;
}

void MainWindow::callbackSteerProofreadingMain(const autoware_can_msgs::SteerProofreading::ConstPtr &msg)
{
	//最小二乗法を用いて倍率と中央値を計算
	/*uint16_t xmin=USHRT_MAX, xmax=0;
	double xave=0, yave=0;
	for(const autoware_can_msgs::MicroBusSHHV shhv : shhv_list_)
	{
		xave += shhv.mechanism_steering_sub;
		yave += shhv.mechanism_steering_main;
		xmin = std::min(xmin, shhv.mechanism_steering_sub);
		xmax = std::max(xmax, shhv.mechanism_steering_sub);
	}
	xave /= shhv_list_.size();
	yave /= shhv_list_.size();
	double sum1=0, sum2=0;
	for(const autoware_can_msgs::MicroBusSHHV shhv : shhv_list_)
	{
		sum1 += (shhv.mechanism_steering_sub - xave) * (shhv.mechanism_steering_main - yave);
		sum2 += (shhv.mechanism_steering_sub - xave) * (shhv.mechanism_steering_sub - xave);
	}
	double slop = sum1 / sum2;
	double inc = yave - slop * xave;

	/*std::stringstream ss;
	ss << std::fixed << std::setprecision(2) << "STEER校正を\n適用しますか？\n倍率：" << msg->magn << "\n中央値：" << msg->actual_center;
	//std::stringstream ss;
	//ss << std::fixed << std::setprecision(2) << "STEER校正を\n適用しますか？\n傾き:" << msg->data[0] << "\n０点:" << msg->data[1];

	int button;
	{
		QMessageBox msgbox(this);
		msgbox.setIcon(QMessageBox::Question);
		msgbox.setWindowTitle(tr("STEER自動校正の確認"));
		msgbox.setText(tr(ss.str().c_str()));
		msgbox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
		msgbox.setDefaultButton(QMessageBox::No);
		msgbox.setButtonText(QMessageBox::Yes, tr("適用"));
		msgbox.setButtonText(QMessageBox::No, tr("CANCEL"));
		QFont font = msgbox.font();
		font.setPointSize(35);
		msgbox.setFont(font);
		button = msgbox.exec();
	}*/
}

void MainWindow::callbackSteerProofreadingBase(const autoware_can_msgs::SteerProofreading::ConstPtr &msg)
{
	std::stringstream ss;
	uint16_t voltage_range_main = msg->max_voltage_main - msg->min_voltage_main;
	uint16_t voltage_range_sub = msg->max_voltage_sub - msg->min_voltage_sub;
	//ss << "MAIN最大電圧：" << msg->max_voltage_main << "\nMAIN最小電圧：" << msg->min_voltage_main << "\nMAIN電圧範囲：" << voltage_range_main;
	ss << "\nSUB最大電圧：" << msg->max_voltage_sub << "\nSUB最小電圧：" << msg->min_voltage_sub << "\nSUB電圧範囲：" << voltage_range_sub;

	dialog_steer_proofreading_sub form(this);
	form.setText(ss.str());
	form.exec();
	int straight_sub_voltage = form.straightSubVoltage();

	if(straight_sub_voltage >= 0)
	{
		rs232_steer_voltage_range_ = voltage_range_sub;
		rs232_steer_voltage_straight_ = (uint16_t)straight_sub_voltage;

		QMessageBox msgbox(this);
		msgbox.setIcon(QMessageBox::Information);
		msgbox.setWindowTitle(tr("OK"));
		msgbox.setText(tr("サブ電圧情報を登録しました"));
		msgbox.setStandardButtons(QMessageBox::Ok);
		msgbox.setDefaultButton(QMessageBox::Ok);
		msgbox.setButtonText(QMessageBox::Ok, tr("OK"));
		QFont font = msgbox.font();
		font.setPointSize(35);
		msgbox.setFont(font);
		msgbox.exec();

		//ステア校正用電圧値の保存
		std::string log_folder_path = ros::package::getPath("runtime_manager") + "/steer_voltage_param";
		std::ofstream ofs_log(log_folder_path, std::ios_base::out);
		if(ofs_log)
		{
			ofs_log << rs232_steer_voltage_range_ << "\n";
			ofs_log << rs232_steer_voltage_straight_;
			ofs_log.close();
		}
	}
}

void MainWindow::callbackSafetyWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
{
	if(msg->waypoints.size() == 0) return;

	read_safety_waypoints_=true;

	int waycou;
	double waypoints_lenght = 0;//探索した範囲のwaypointsの長さ
	for(waycou=1; waycou<msg->waypoints.size(); waycou++)
	{
		const geometry_msgs::Point &way_po = msg->waypoints[waycou].pose.pose.position;
		const geometry_msgs::Point &way_po_prev = msg->waypoints[waycou-1].pose.pose.position;
		waypoints_lenght += euclideanDistanceXY(way_po, way_po_prev);

		if(msg->waypoints[waycou].waypoint_param.oncoming_stop_line > 0)//対向車用停止線が存在するか？
		{
			int16_t exploration_range = msg->waypoints[waycou].waypoint_param.oncoming_stop_line;//探索射程

			if(oncoming_obs_.existence == true)//対向車が存在するか？
			{
				std::cout << "taikousya" << std::endl;
				const geometry_msgs::Point &base_po = msg->waypoints[0].pose.pose.position;
				const geometry_msgs::Point &line_po = msg->waypoints[waycou].pose.pose.position;
				//double onc_dis = euclideanDistanceXY(base_po, oncoming_obs_.map_pos);
				//double line_dis = euclideanDistanceXY(base_po, line_po);
				double line_obs_dis = euclideanDistanceXY(line_po, oncoming_obs_.map_pos);//対向車停止線と対向車の距離
				double base_obs_dis = euclideanDistanceXY(base_po, oncoming_obs_.map_pos);//waypoint0点と対向車の距離
				double base_line_dis = euclideanDistanceXY(base_po, line_po);//waypoint0点と対向車停止線の距離

				std::cout << "line_obs_dis," << line_obs_dis << "<" << exploration_range << std::endl;
				std::cout << "base_obs_dis," << base_obs_dis << ">" << base_line_dis << std::endl;
				std::cout << "oncoming_stop_id_," << oncoming_stop_id_ << "!=" << msg->waypoints[waycou].waypoint_param.id << std::endl;
				//if(oncoming_stop_id_ != msg->waypoints[waycou].waypoint_param.id && onc_dis < line_dis)
				if(line_obs_dis < exploration_range && //停止線基準での対向車との距離が
					base_obs_dis > base_line_dis &&
					oncoming_stop_id_ != msg->waypoints[waycou].waypoint_param.id)
				{
					dialog_popup_signal_->setPopupSignal(autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_TAIKOSYATUKAMATI,
						panel_front_text_[autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_TAIKOSYATUKAMATI-1].toStdString(), false);
					dialog_popup_signal_->setWaypointID(msg->waypoints[waycou].waypoint_param.id);
					dialog_popup_signal_->show();
					oncoming_stop_id_ = msg->waypoints[waycou].waypoint_param.id;
				}
			}
			break;
		}
	}
	if(waycou == msg->waypoints.size())
	{
		oncoming_stop_id_ = -1;
	}
}

void MainWindow::callbackGnssTime(const autoware_system_msgs::Date &msg)
{
	gnss_time_ = msg;
}

void MainWindow::callbackAutomodeMileage(const std_msgs::Float64 &msg)
{
	automode_mileage_ = msg.data;
}

void MainWindow::callbackVehicleCmd(const autoware_msgs::VehicleCmd &msg)
{
	vehicle_cmd_ = msg;
}

void MainWindow::callbackCmdSelect(const std_msgs::Int32 &msg)
{
	cmd_select_ = msg.data;
}

void MainWindow::callbackLoadName(const autoware_msgs::WaypointsSerialNumLaunch &msg)
{
	/*std::vector<std::string> strs = split(msg.data);
	if(strs.size() != 2)
	{
		ui->tx4_load_name->setText("NONE");
	}
	else
	{
		std::stringstream ss;
		ss << strs[0] << "\n↓\n" << strs[1];
		ui->tx4_load_name->setText(ss.str().c_str());
	}*/

	/*std::string text = msg.route_current + "\n↓\n" + msg.route_next;
	ui->tx4_load_name->setText(text.c_str());

	if(msg.toAnext == 0) ui->bt4_nextA->setEnabled(false);
	else ui->bt4_nextA->setEnabled(true);
	if(msg.toAback == 0) ui->bt4_backA->setEnabled(false);
	else ui->bt4_backA->setEnabled(true);
	if(msg.toBnext == 0) ui->bt4_nextB->setEnabled(false);
	else ui->bt4_nextB->setEnabled(true);
	if(msg.toBback == 0) ui->bt4_backB->setEnabled(false);
	else ui->bt4_backB->setEnabled(true);

	waypoints_serial_num_ = msg;*/

	std::stringstream ss;
	//ss << (auto_route_loop_count_.loop - 1) * 2 << "便\n";
	//ss << (auto_route_loop_count_.loop) << "ライド目\n";
	std::string text = ss.str() + msg.route_current + "\n↓\n" + msg.route_next;
	ui->tx4_load_name->setText(text.c_str());

	/*if(msg.toAnext == "") ui->bt4_nextA->setEnabled(false);
	else ui->bt4_nextA->setEnabled(true);
	if(msg.toAback == "") ui->bt4_backA->setEnabled(false);
	else ui->bt4_backA->setEnabled(true);
	if(msg.toBnext == "") ui->bt4_nextB->setEnabled(false);
	else ui->bt4_nextB->setEnabled(true);
	if(msg.toBback == "") ui->bt4_backB->setEnabled(false);
	else ui->bt4_backB->setEnabled(true);*/

	waypoints_serial_num_ = msg;
}

void MainWindow::callbackBaseWaypoints(const autoware_msgs::LaneArray &msg)
{
}

void MainWindow::callbackCurrentVelocity(const geometry_msgs::TwistStamped &msg)
{
	localizer_time_diff_ = msg.header.stamp - current_velocity_.header.stamp;
	current_velocity_ = msg;
	//current_velocity_.header.stamp = ros::Time::now();
}

void MainWindow::callbackAutoRouteLoopCount(const autoware_msgs::AutoRouteLoopCount::ConstPtr &msg)
{
	auto_route_loop_count_ = *msg;
}

void MainWindow::callbackSteerCorrectionWriteReturn(const std_msgs::String::ConstPtr &msg)
{
	QMessageBox msgbox(this);
	msgbox.setIcon(QMessageBox::Information);
	msgbox.setWindowTitle(tr("ステア調整終了"));
	msgbox.setText(tr(msg->data.c_str()));
	msgbox.setStandardButtons(QMessageBox::Ok);
	msgbox.setDefaultButton(QMessageBox::Ok);
	msgbox.setButtonText(QMessageBox::Ok, tr("OK"));
	int button = msgbox.exec();
}

void MainWindow::callbackAcc(const std_msgs::Float64 &msg)
{
	acc_ = msg.data;
}

void MainWindow::callbackPanelRead(const std_msgs::UInt16::ConstPtr &msg)
{
	panel_view_front_ = (uint8_t)(msg->data >> 8 & 0x00FF);
	panel_view_back_ = (uint8_t)msg->data;
}

void MainWindow::callbackPopupSignal(const autoware_msgs::InterfacePopupSignal::ConstPtr &msg)
{
	if(msg->popup_signal > autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO
		&& msg->popup_signal <= autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_HOREISOKUDOJUNSYU)
	{
		dialog_popup_signal_->setPopupSignal(msg->popup_signal, panel_front_text_[msg->popup_signal-1].toStdString(), false);
		dialog_popup_signal_->show();
	}
	else if(msg->popup_signal == autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO)
	{
		dialog_popup_signal_->setPopupSignal(autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO, "リセット しますか？", false);
		dialog_popup_signal_->show();
	}
	else if(msg->popup_signal == autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_LEFT)
	{
		dialog_popup_signal_->setPopupSignal(autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_LEFT, "左ウィンカー", false);
		dialog_popup_signal_->show();
	}
	else if(msg->popup_signal == autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_RIGHT)
	{
		dialog_popup_signal_->setPopupSignal(autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_RIGHT, "右ウィンカー", false);
		dialog_popup_signal_->show();
	}
	else if(msg->popup_signal == autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_STOP)
	{
		dialog_popup_signal_->blinkerCessation();
		publish_blinker_stop();
	}
}

void MainWindow::callbackFrontMobileye(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg)
{
	mobileye_front_car_ = *msg;
}

void MainWindow::callbackMobileyeCmdParam(const autoware_msgs::MobileyeCmdParam::ConstPtr &msg)
{
	mobileye_cmd_param_ = *msg;
}

void MainWindow::callbackTrackingType(const std_msgs::String::ConstPtr &msg)
{
	tracking_type_ = msg->data;
}

void MainWindow::callbackVoxelGirdFilter(const autoware_config_msgs::ConfigVoxelGridFilter::ConstPtr& msg)
{
	config_voxel_grid_filter_ = *msg;
}

void MainWindow::publish_emergency_clear()
{
	std_msgs::Empty msg;
	pub_unlock_.publish(msg);
}

void MainWindow::publish_Dmode_manual()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_drive_mode_.publish(msg);
}

void MainWindow::publish_Dmode_program()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_drive_mode_.publish(msg);
}

void MainWindow::publish_Dmode_velocity()
{
	std_msgs::Int8 msg;
	msg.data = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY;
	pub_drive_control_.publish(msg);
}

void MainWindow::publish_Dmode_stroke()
{
	std_msgs::Int8 msg;
	msg.data = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE;
	pub_drive_control_.publish(msg);
}

void MainWindow::publish_Dmode_input_direct()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_drive_input_.publish(msg);
}

void MainWindow::publish_Dmode_input_auto()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_drive_input_.publish(msg);
}

void MainWindow::publish_Smode_manual()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_steer_mode_.publish(msg);
}

void MainWindow::publish_Smode_program()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_steer_mode_.publish(msg);
}

void MainWindow::publish_Smode_input_direct()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_steer_input_.publish(msg);
}

void MainWindow::publish_Smode_input_auto()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_steer_input_.publish(msg);
}

void MainWindow::publish_drive_clutch_connect()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_drive_clutch_.publish(msg);
}

void MainWindow::publish_drive_clutch_cut()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_drive_clutch_.publish(msg);
}

void MainWindow::publish_steer_clutch_connect()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_steer_clutch_.publish(msg);
}

void MainWindow::publish_steer_clutch_cut()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_steer_clutch_.publish(msg);
}

void MainWindow::publish_blinker_right()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_blinker_right_.publish(msg);
	prev_blinker_ = 2;
}

void MainWindow::publish_blinker_left()
{
	std_msgs::Bool msg;
	msg.data = true;
	pub_blinker_left_.publish(msg);
	prev_blinker_ = 1;
}

void MainWindow::publish_blinker_stop()
{
	//std_msgs::Bool msg;
	//msg.data = true;
	//pub_blinker_stop_.publish(msg);
	std_msgs::Bool msg;
	msg.data = false;
	if(prev_blinker_ == 1) pub_blinker_left_.publish(msg);
	else if(prev_blinker_ == 2) pub_blinker_right_.publish(msg);
	prev_blinker_ = 0;
}

void MainWindow::publish_use_safety_localizer()
{
	std_msgs::Bool msg;
	msg.data = ui->cb_use_localizer_safety->isChecked();
	pub_use_safety_localizer_.publish(msg);
}

void MainWindow::publish_use_error_check()
{
	std_msgs::Bool msg;
	msg.data = ui->cb_error_check->isChecked();
	pub_use_error_check_.publish(msg);
}

void MainWindow::rosbag_write()
{
	if(rosbag_write_flag_ == false && record_topic_list_ != "")
	{
		std::string rosbag_cmd;
		std::string  record_topic = stringReplace(record_topic_list_, ',', ' ');

		time_t nowtime = time(NULL);
		tm* date = localtime(&nowtime);
		int year = date->tm_year-100+2000;
		int mou = date->tm_mon+1;
		int day = date->tm_mday;
		std::stringstream ss_log_folder_day;
		ss_log_folder_day << log_folder_ << "/" << std::setfill('0') << std::right << std::setw(2) << year << "_" << std::setw(2)<< mou << "_" << std::setw(2) << day;
		if(existFile(ss_log_folder_day.str().c_str()) == 0) mkdir(ss_log_folder_day.str().c_str(), 0777);
		std::cout << "mkdir," << ss_log_folder_day.str() << std::endl;
		//" __name:=rosbag_record -O "
		std::stringstream ss_bag;
		ss_bag << rostime2date(ros::Time::now()) << ".bag";
		rosbag_cmd += std::string("rosbag record ") + record_topic + " -O " + ss_log_folder_day.str() + "/" + ss_bag.str() + " __name:=rosbag_record &";
		std::cout << rosbag_cmd << std::endl;
		system(rosbag_cmd.c_str());
		rosbag_write_flag_ = true;
	}
}

void MainWindow::publish_log_write()
{
	std_msgs::Bool msg;
	//if(log_write_flag_ == false) msg.data = true;
	//else msg.data = false;
	msg.data = true;
	pub_log_write_.publish(msg);
	ui->bt2_log_write->setPalette(palette_logbt_on_);
	ui->bt2_log_stop->setPalette(palette_logbt_off_);
	rosbag_write();
}

void MainWindow::rosbag_stop()
{
	if(rosbag_write_flag_ == true)
	{
		system("rosnode kill /rosbag_record &");
		rosbag_write_flag_ = false;
	}
}

void MainWindow::publish_log_stop()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_log_write_.publish(msg);
	ui->bt2_log_write->setPalette(palette_logbt_off_);
	ui->bt2_log_stop->setPalette(palette_logbt_on_);
	rosbag_stop();
}

void MainWindow::publish_specified_speed()
{
	use_specified_cmd_ = true;
}

void MainWindow::publish_specified_speed_stop()
{
	use_specified_cmd_ = false;
}

void MainWindow::click_error_text_reset()
{
	std_msgs::Bool msg;
	msg.data = false;
	pub_error_lock_.publish(msg);

	error_text_lock_ = false;
	ui->tx_error_text->setText("");
	ui->tx2_error_text->setText("");
}

std::string MainWindow::gnss_time_str()
{
	std::stringstream str;
	str << gnss_time_.year << ":" << +gnss_time_.month << ":" << +gnss_time_.day << ":" << +gnss_time_.hour << ":" << +gnss_time_.min << ":" << gnss_time_.sec;
	return str.str();
}

void MainWindow::click_signal_time()
{
	std::string time_str = gnss_time_str();
	double time = gnss_time_.hour*60.0*60.0 + gnss_time_.min*60.0 + gnss_time_.sec;

	QString red_green_text = ui->tx3_signal_red_green_time->toPlainText();
	QString green_yellow_text = ui->tx3_signal_green_yellow_time->toPlainText();
	QString yellow_red_text = ui->tx3_signal_yellow_red_time->toPlainText();
	QString red_green_text2 = ui->tx3_signal_red_green_time_2->toPlainText();
	if(red_green_text == "")
		{ui->tx3_signal_red_green_time->setText(time_str.c_str()); signal_red_green_time_ = time;}
	else if(green_yellow_text == "")
		{ui->tx3_signal_green_yellow_time->setText(time_str.c_str()); signal_green_yellow_time_ = time;}
	else if(yellow_red_text == "")
		{ui->tx3_signal_yellow_red_time->setText(time_str.c_str()); signal_yellow_red_time_ = time;}
	else if(red_green_text2 == "")
		{ui->tx3_signal_red_green_time_2->setText(time_str.c_str()); signal_red_green_time2_ = time;}

	if(signal_red_green_time_ != 0 && signal_green_yellow_time_)
	{
		std::stringstream str;
		str << signal_green_yellow_time_ - signal_red_green_time_;
		ui->tx3_signal_red_green_difference->setText(str.str().c_str());
	}
	if(signal_green_yellow_time_ != 0 && signal_yellow_red_time_)
	{
		std::stringstream str;
		str << signal_yellow_red_time_ - signal_green_yellow_time_;
		ui->tx3_signal_green_yellow_difference->setText(str.str().c_str());
	}
	if(signal_yellow_red_time_ != 0 && signal_red_green_time2_)
	{
		std::stringstream str;
		str << signal_red_green_time2_ - signal_yellow_red_time_;
		ui->tx3_signal_yellow_red_difference->setText(str.str().c_str());
	}
}

void MainWindow::click_log_folder()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("フォルダ選択画面"),
									  QStandardPaths::writableLocation(QStandardPaths::DesktopLocation));
	if(path != "")
	{
		log_folder_ = path.toStdString();
		std_msgs::String str;
		str.data = log_folder_;
		pub_log_folder_.publish(str);
	}
}

void MainWindow::click_signal_time_clear()
{
	ui->tx3_signal_red_green_time->setText("");
	ui->tx3_signal_green_yellow_time->setText("");
	ui->tx3_signal_yellow_red_time->setText("");
	ui->tx3_signal_red_green_time_2->setText("");
	ui->tx3_signal_green_yellow_difference->setText("");
	ui->tx3_signal_yellow_red_difference->setText("");
	ui->tx3_signal_red_green_difference->setText("");
	signal_red_green_time_ = signal_green_yellow_time_ = signal_yellow_red_time_ = signal_red_green_time2_ = 0;
}

void MainWindow::click_load_nextA()
{
	publish_log_stop();
	//killWaypointsNode();
	std_msgs::Bool msg;
	msg.data = true;
	pub_auto_route_increment_.publish(msg);
	/*if(waypoints_serial_num_.toAback == "" &&
	   waypoints_serial_num_.toAnext == "" &&
	   waypoints_serial_num_.toBback == "" &&
	   waypoints_serial_num_.toBnext == "")//連番読み込みがされていない場合、1A.launchを読み込む
	{
		QModelIndexList index_list = ui->list_read_launch->selectionModel()->selectedIndexes();
		QModelIndex index = index_list[0];
		runWaypointsNode(select_launch_file_[index.row()]);    
	}
	else runWaypointsNode(waypoints_serial_num_.toAnext);*/
}

void MainWindow::click_load_backA()
{
	publish_log_stop();
	//killWaypointsNode();
	std_msgs::Bool msg;
	msg.data = false;
	pub_auto_route_increment_.publish(msg);
	//runWaypointsNode(waypoints_serial_num_.toAback);
}

void MainWindow::click_load_nextB()
{
	//publish_log_stop();
	//killWaypointsNode();
	//runWaypointsNode(waypoints_serial_num_.toBnext);
}

void MainWindow::click_load_backB()
{
	//publish_log_stop();
	//killWaypointsNode();
	//runWaypointsNode(waypoints_serial_num_.toBback);
}

void MainWindow::click_auto_change_loop_next()
{
	publish_log_stop();
	//killWaypointsNode();
	std_msgs::Bool msg;
	msg.data = true;
	pub_auto_route_loop_next_.publish(msg);
}

void MainWindow::click_auto_change_loop_back()
{
	publish_log_stop();
	//killWaypointsNode();
	std_msgs::Bool msg;
	msg.data = false;
	pub_auto_route_loop_next_.publish(msg);
}

void MainWindow::specified_speed_add(double add)
{
	char *str = ui->li5_can_send_velocity_->text().toUtf8().data();
	double speed = atof(str) + add;
	int maxval = ui->slider_can_send_velocity->maximum();
	int minval = ui->slider_can_send_velocity->minimum();
	if(speed > maxval) speed = maxval;
	if(speed < minval) speed = minval;
	char buf[10];
	sprintf(buf, "%2.3f", speed);
	ui->li5_can_send_velocity_->setText(buf);
}

void MainWindow::specified_deg_add(double add)
{
	char *str = ui->li5_can_send_deg->text().toUtf8().data();
	double deg = atof(str) + add;
	int maxval = ui->slider_can_send_deg->maximum();
	int minval = ui->slider_can_send_deg->minimum();
	if(deg > maxval) deg = maxval;
	if(deg < minval) deg = minval;
	char buf[10];
	sprintf(buf, "%2.3f", deg);
	ui->li5_can_send_deg->setText(buf);
}

void MainWindow::click_specified_speed_plus1()
{
	specified_speed_add(1);
}

void MainWindow::click_specified_speed_minus1()
{
	specified_speed_add(-1);
}

void MainWindow::click_specified_speed_plus5()
{
	specified_speed_add(5);
}

void MainWindow::click_specified_speed_minus5()
{
	specified_speed_add(-5);
}

void MainWindow::click_specified_deg_plus1()
{
	specified_deg_add(1);
}

void MainWindow::click_specified_deg_minus1()
{
	specified_deg_add(-1);
}

void MainWindow::click_specified_deg_plus5()
{
	specified_deg_add(5);
}

void MainWindow::click_specified_deg_minus5()
{
	specified_deg_add(-5);
}

void MainWindow::click_specified_deg_plus_any()
{
	std::string str = ui->li5_can_send_deg_any->text().toStdString();
	double val = atof(str.c_str());
	specified_deg_add(val);
}

void MainWindow::click_specified_deg_minus_any()
{
	std::string str = ui->li5_can_send_deg_any->text().toStdString();
	double val = atof(str.c_str());
	specified_deg_add(-val);
}

void MainWindow::click_rviz_restart()
{
	system("rosrun rviz_restart rviz_restart &");
}

void MainWindow::check_use_auto_chenge(bool flag)
{
	if(flag)
		system("rosrun waypoint_maker auto_waypoints_change &");
	else
		system("rosnode kill /auto_waypoints_change &");
}

void MainWindow::check_use_auto_log(bool flag)
{
	std_msgs::Bool msg;
	msg.data = flag;
	pub_auto_log_.publish(msg);
}

void MainWindow::click_steer_plus()
{
	std::string str = ui->li2_steer_plus_interface->text().toStdString();
	int16_t val = atoi(str.c_str());
	std_msgs::Int16 msg_val;
	msg_val.data = val;
	pub_steer_plus_.publish(msg_val);
}

void MainWindow::click_steer_plus_ave()
{
	use_steer_plus_sum_ = true;
	steer_plus_sum_ = 0.0;
	steer_plus_sum_count_ = 0;
}

void MainWindow::click_arena_gain_send()
{
	std::string str = ui->li3_arena_gain->text().toStdString();
	double val = atof(str.c_str());
	std_msgs::Float64 msg_val;
	msg_val.data = val;
	pub_arena_gain_.publish(msg_val);
}

void MainWindow::click_steer_plus_ave_stop()
{
	use_steer_plus_sum_ = false;
}

void MainWindow::click_steer_P10()
{
	std::string str = ui->li2_steer_plus_interface->text().toStdString();
	int16_t val = atoi(str.c_str());
	val += 10;
	std::stringstream ss;
	ss << val;
	ui->li2_steer_plus_interface->setText(ss.str().c_str());
	std_msgs::Int16 msg_val;
	msg_val.data = val;
	pub_steer_plus_.publish(msg_val);
}

void MainWindow::click_steer_M10()
{
	std::string str = ui->li2_steer_plus_interface->text().toStdString();
	int16_t val = atoi(str.c_str());
	val -= 10;
	std::stringstream ss;
	ss << val;
	ui->li2_steer_plus_interface->setText(ss.str().c_str());
	std_msgs::Int16 msg_val;
	msg_val.data = val;
	pub_steer_plus_.publish(msg_val);
}

void MainWindow::click_auto_shutdown()
{
	QMessageBox msgbox(this);
	msgbox.setIcon(QMessageBox::Question);
	msgbox.setWindowTitle(tr("確認"));
	msgbox.setText(tr("終了してよろしいですか？"));
	msgbox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgbox.setDefaultButton(QMessageBox::No);
	msgbox.setButtonText(QMessageBox::Yes, tr("終了"));
	msgbox.setButtonText(QMessageBox::No, tr("戻る"));
	int button = msgbox.exec();
	if(button == QMessageBox::Yes)
	{
		system("rosnode kill -a");
		system("shutdown -h now &");
	}
}

void MainWindow::steer_sub_interface_correction_write_end(const bool confirm)
{
	std_msgs::Bool msg;
	msg.data = confirm;
	pub_steer_sub_interface_correction_write_end_.publish(msg);//ステア調整書き込みノード終了
}

//パネル応答popupからの返答を受け取る関数
void MainWindow::popup_signal_interface_ret(const bool ok, const uint8_t signal, const uint32_t waypoint_id)
{
	switch (signal)
	{
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_ZIDOUNTENZISSITYU:
		panel_flag_front_.set_zidountenzissityu(ok);
		panel_flag_back_.set_zidountenzissityu(ok);
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_SAKINIDOUZO:
		panel_flag_front_.set_sakinidouzo(ok);
		panel_flag_back_.set_sakinidouzo(ok);
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_SAITAMAKOGYODAIGAKU:
		panel_flag_front_.set_saitamakogyodaigaku(ok);
		panel_flag_back_.set_saitamakogyodaigaku(ok);
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_USETUSIMASU:
		panel_flag_front_.set_usetusimasu(ok);
		panel_flag_back_.set_usetusimasu(ok);
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_MAMONAKUAKA:
		panel_flag_front_.set_mamonakuaka(ok);
		panel_flag_back_.set_mamonakuaka(ok);
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_TAIKOSYATUKAMATI:
		//panel_flag_front_.set_taikosyatukamati(ok);
		panel_flag_front_.set_sakinidouzo(ok);
		panel_flag_back_.set_taikosyatukamati(ok);
		if(ok)
		{
			std_msgs::UInt32 permission_msg;
			permission_msg.data = waypoint_id;
			pub_oncoming_permission_.publish(permission_msg);
			dialog_popup_signal_->setPopupSignal(autoware_msgs::InterfacePopupSignal::SIGNAL_SYUPATUSIMASU, "出発 しますか？", true);
			dialog_popup_signal_->setWaypointID(waypoint_id);
			dialog_popup_signal_->show();
		}
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_HOREISOKUDOJUNSYU:
		panel_flag_front_.set_horeisokudojunsyu(ok);
		panel_flag_back_.set_horeisokudojunsyu(ok);
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_SYUPATUSIMASU:
		{
			panel_flag_front_.set_sakinidouzo(false);
			panel_flag_back_.set_taikosyatukamati(false);
			std_msgs::UInt32 permission_msg;
			permission_msg.data = 0;
			pub_oncoming_permission_.publish(permission_msg);
		}
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO:
		if(ok == true)
		{
			panel_flag_front_.set_reset();
			panel_flag_back_.set_reset();
		}
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_LEFT:
		publish_blinker_left();
		break;
	case autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_RIGHT:
		publish_blinker_left();
		break;
	}

	autoware_msgs::InterfacePopupReturn ret_msg;
	ret_msg.header.stamp = ros::Time::now();
	ret_msg.popup_sig = signal;
	ret_msg.ok = ok;
	pub_popup_return_.publish(ret_msg);
}

void MainWindow::click_call_dialog_driving_adjustment()
{
	if(dialog_driving_adjustment_->isHidden())// && ui->cb4_dialog_driving_adjustment->isChecked())
		dialog_driving_adjustment_show();
	/*else
	{
		steer_sub_interface_correction_write_end();
		dialog_driving_adjustment_->close();
		dialog_driving_adjustment_->valueClear();
	}*/
}

void MainWindow::click_steer_proofreading_main()
{
	system("roslaunch vehicle_socket steer_proofreading.launch use_voltage:=1 &");
}

void MainWindow::click_steer_proofreading_base()
{
	system("roslaunch vehicle_socket steer_proofreading.launch use_voltage:=0 &");
}

void MainWindow::click_rosbag()
{
	Dialog_rosbag form(this);

	//subscribe topic一覧を取得
	ros::master::V_TopicInfo master_topics;
	if(ros::master::getTopics(master_topics))
	{
		for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
		{
			const ros::master::TopicInfo& info = *it;
			form.setSubscribeTopicName(info.name);
		}
	}
	else
	{
		QMessageBox msgBox(this);
		std::string error_str = "SUBSCRIBE TOPIC 一覧が取得できません";
		msgBox.setText(error_str.c_str());
		msgBox.setWindowTitle("error");
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.setDefaultButton(QMessageBox::Ok);
		msgBox.exec();
	}

	//record topic一覧を取得
	std::vector<std::string> record_topics = split(record_topic_list_, ',');
	for(const std::string topic : record_topics)
		form.setRecordTopicName(topic);

	form.exec();
	if(form.ok())
	{
		record_topic_list_ = form.recordTopicList();
		std_msgs::String str;
		str.data = record_topic_list_;
		pub_record_topic_list_.publish(str);

		//record topicを保存
		{
			std::string record_path = ros::package::getPath("runtime_manager") + "/record_topic";
			std::ofstream ofs_record(record_path, std::ios_base::out);
			if(ofs_record)
			{
				std::vector<std::string> topics = split(record_topic_list_, ' ');
				for(int i=0; i<topics.size(); i++)
				{
					ofs_record << topics[i];
					if(i != topics.size()-1) ofs_record << '\n';
				}
			}
			ofs_record.close();
		}
	}
}

void MainWindow::click_track_excess_acc()
{
	std::ofstream ofs("/tmp/track_irregular", std::ios_base::app);
	ofs << waypoint_param_.id << "," << "過加速\n";
	ofs.close();
}

void MainWindow::click_track_excess_stop()
{
	std::ofstream ofs("/tmp/track_irregular", std::ios_base::app);
	ofs << waypoint_param_.id << "," << "過停止\n";
	ofs.close();
}

void MainWindow::click_yure()
{
	std::ofstream ofs("/tmp/track_irregular", std::ios_base::app);
	ofs << waypoint_param_.id << "," << "ゆれ\n";
	ofs.close();
}

void MainWindow::slide_specified_speed(int val)
{
	char buf[10];
	sprintf(buf, "%d", val);
	ui->li5_can_send_velocity_->setText(buf);
}

void MainWindow::slide_specified_deg(int val)
{
	char buf[10];
	sprintf(buf, "%d", val);
	ui->li5_can_send_deg->setText(buf);
}

void MainWindow::car_target_deceleration_change(double val)
{
	std_msgs::Float64 msg;
	msg.data = val;
	pub_car_target_deceleration_.publish(msg);
}

void MainWindow::currentItemchange_list_read_launch(QListWidgetItem* new_item,QListWidgetItem* prev_item)
{
	std::string str = new_item->text().toStdString();
	for(int i=0; i<select_launch_text_.size(); i++)
	{
		std::string select_route = select_launch_text_[i].toStdString();
		if(select_route == str)
		{
			std::vector<std::string> route = select_launch_[i];
			autoware_msgs::AutoRouteList msg;
			msg.header.stamp = ros::Time::now();
			msg.list = route;
			pub_auto_route_list_.publish(msg);
			break;
		}
	}
}
