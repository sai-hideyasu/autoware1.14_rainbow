#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "dialog_driving_adjustment.h"
#include "dialog_popup_signal.h"
#include "panel_view_flag.h"
#include "dialog_steer_proofreading_sub.h"
#include "dialog_rosbag.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iomanip>
#include <fstream>
#include <sys/stat.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <mobileye_560_660_msgs/AftermarketLane.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_msgs/LocalizerMatchStat.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_msgs/DifferenceToWaypointDistance.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_can_msgs/MicroBusCanVelocityParam.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_system_msgs/Date.h>
#include <autoware_msgs/AutoRouteList.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/WaypointsSerialNumLaunch.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/AutoRouteLoopCount.h>
#include <autoware_msgs/SteerSubInterfaceCorrection.h>
#include <autoware_msgs/InterfacePopupSignal.h>
#include <autoware_msgs/InterfacePopupReturn.h>
#include <autoware_msgs/NearOncomingObs.h>
#include <autoware_msgs/TransformMobileyeObstacle.h>
#include <autoware_msgs/MobileyeCmdParam.h>
#include <autoware_can_msgs/SteerProofreading.h>
#include <QFileDialog>
#include <QStandardPaths>
#include <QMessageBox>
#include <QListWidgetItem>

namespace Ui {
class MainWindow;
}

struct LaneName
{
	std::string launch_name_;
	std::string starting_point_;
	std::string arrival_point_;
};

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(ros::NodeHandle nh, ros::NodeHandle p_nh, QWidget *parent = nullptr);
	~MainWindow();

	void window_updata();

	void steer_sub_interface_correction_write_end(const bool confirm);//インターフェースから送るステア調整を終了してファイルに書き込む指令を送る関数
	void popup_signal_interface_ret(const bool cancel, const uint8_t signal, const uint32_t waypoint_id);//車両パネル指令を受け取る関数
private:
	double wheelrad_to_steering_can_value_left_ = 25009.6727514125;//liesse 20935.4958411006;//cmdのwheel指令をcanのハンドル指令に変換する係数(左回り用)
	double wheelrad_to_steering_can_value_right_ = 26765.9140133745;//liesse 20791.4464661611;//cmdのwheel指令をcanのハンドル指令に変換する係数(右回り用)

	const short PEDAL_VOLTAGE_CENTER_ = 1024;//ペダルをニュートラルにしている際の電圧値
	static const int TRAFFIC_LIGHT_RED = 0;
	static const int TRAFFIC_LIGHT_GREEN  = 1;
	static const int TRAFFIC_LIGHT_UNKNOWN = 2;

	//static const int LOAD_NAME_MAX = 7;//tab4のlaunchファイルの連番最大数
	//int load_name_count_;//tab4_launchファイルの現在の連番
	autoware_msgs::WaypointsSerialNumLaunch waypoints_serial_num_;//tab4の経路launchファイルの連番情報

	Ui::MainWindow *ui;

	ros::NodeHandle nh_, private_nh_;

	ros::Publisher pub_unlock_;//デバイス立ち上がり時のLOCKを解除
	ros::Publisher pub_drive_mode_, pub_steer_mode_;//autoモードとmanualモードのチェンジ
	ros::Publisher pub_drive_control_;//driveのコントロールモード(velocity操作とstroke操作の切り替え)
	ros::Publisher pub_drive_input_, pub_steer_input_;//programモード時の自動入力と手動入力の切り替え
	ros::Publisher pub_drive_clutch_, pub_steer_clutch_;//クラッチの状態変更フラグ
	ros::Publisher pub_blinker_right_, pub_blinker_left_, pub_blinker_stop_; //ウィンカー
	ros::Publisher pub_error_lock_;//エラーがでている場合、canアプリにロック情報を送る
	ros::Publisher pub_use_safety_localizer_;//localizer関連のセーフティのチェック
	ros::Publisher pub_log_write_, pub_log_stop, pub_log_folder_;//ログ出力通知
	ros::Publisher pub_use_error_check_;//距離のフェイルセーブを使用するか
	ros::Publisher pub_vehicle_cmd_;//canに固定の指定速度指令を送る
	ros::Publisher pub_steer_plus_;//steer補正をpublish
	ros::Publisher pub_auto_log_;//log自動取得flag
	ros::Publisher pub_arena_gain_;//arenaカメラのゲイン値を送信
	ros::Publisher pub_auto_route_list_;//経路自動切り替えの一覧を送信
	ros::Publisher pub_auto_route_increment_;//経路自動切り替えを１つすすめる、または戻す
	ros::Publisher pub_auto_route_loop_next_;//経路自動切り替えの便を１つすすめる、または戻す
	ros::Publisher pub_steer_sub_interface_correction_;//インターフェースから送るステア調整値
	ros::Publisher pub_steer_sub_interface_correction_write_end_;//インターフェースから送るステア調整を終了してファイルに書き込む指令を送る
	ros::Publisher pub_change_max_speed_;//速度キャップの変更
	ros::Publisher pub_panel_view_;//車両パネル表示
	ros::Publisher pub_panel_view_reset_;//パネル標示のリセット
	ros::Publisher pub_popup_return_;//パネル用popup処理の返答
	ros::Publisher pub_oncoming_permission_;//対向車待ちの停止許可をするか？
	ros::Publisher pub_record_topic_list_;//rosbag recordするトピック一覧(スペース区切り)
	ros::Publisher pub_car_target_deceleration_;//前方車両追跡時に最低車間距離

	ros::Subscriber sub_can501_, sub_can502_, sub_can503_;//マイクロバスcanのID501,502
	ros::Subscriber sub_can_status_;//canステータス情報 
	ros::Subscriber sub_distance_angular_check_, sub_distance_angular_check_ndt_, sub_distance_angular_check_ekf_, sub_distance_angular_check_gnss_;//経路と自車位置のチェック用
	ros::Subscriber sub_config_;
	ros::Subscriber sub_localizer_select_;//localizerの遷移状態 
	ros::Subscriber sub_localizer_match_stat_;//localizerのマッチング状態
	ros::Subscriber sub_can_velocity_param_;//canの速度情報
	ros::Subscriber sub_stopper_distance_;//停止線の位置情報
	ros::Subscriber sub_waypoint_param_;//経路の埋め込み情報
	ros::Subscriber sub_imu_;//IMU情報
	ros::Subscriber sub_gnss_pose_, sub_gnss_deviation_, sub_ndt_stat_, sub_gnss_stat_, sub_ndt_stat_string_;
	ros::Subscriber sub_stroke_routine_;
	ros::Subscriber sub_mobileye_frame_;//mobileyeからのcanの生データ
	ros::Subscriber sub_gnss_time_;//gnssの時間
	ros::Subscriber sub_light_color_;//信号色
	ros::Subscriber sub_period_signal_takeover_;//定時信号での緑でのテイクオーバー
	ros::Subscriber sub_signal_change_time_;//次の信号の切り替わり時間
	ros::Subscriber sub_automode_mileage_;//自動走行時の距離
	ros::Subscriber sub_vehicle_cmd_;//canで処理される速度とステアのコマンド
	ros::Subscriber sub_cmd_select_;//ctrl_rawとtwist_rawをpublishしているノードの種類
	ros::Subscriber sub_load_name_;//waypointを読み込むlaunchファイルでload_nameトピック(std_msgs/String)を投げると、その文字列を表示(場所確認用)
	ros::Subscriber sub_base_waypoints_;//base_waypoints(global waypointの最終情報)がpublishされている場合、tab4でOKを出す
	ros::Subscriber sub_current_velocity_;
	ros::Subscriber sub_acc_;//加速度
	ros::Subscriber sub_auto_route_loop_count_;//自動経路切り替えの周回情報
	ros::Subscriber sub_safety_waypoints_;
	ros::Subscriber sub_steer_correction_write_return_;//ステア調整ノードからの終了返答
	ros::Subscriber sub_panel_read_;//車両のパネル表示状態
	ros::Subscriber sub_popup_signal_;//運転手確認用ポップアップ画面表示しれい
	ros::Subscriber sub_oncoming_obs_;//対向車が存在するか？
	ros::Subscriber sub_steer_proofreading_main_;//steer自動校正通知(MAIN)
	ros::Subscriber sub_steer_proofreading_base_;//steer自動校正通知(BASE)
	ros::Subscriber sub_front_mobileye_;//mobileyeの前方車両情報
	ros::Subscriber sub_mobileye_cmd_param_;//mobileye_trackerノードからpublishされる追跡用情報
	ros::Subscriber sub_tracking_type_;//前方車両追跡時の追従方法

	void callbackCan501(const autoware_can_msgs::MicroBusCan501 &msg);//マイコン応答ID501
	void callbackCan502(const autoware_can_msgs::MicroBusCan502 &msg);//マイコン応答ID502
	void callbackCan503(const autoware_can_msgs::MicroBusCan503 &msg);//マイコン応答ID502
	void callbackCanStatus(const autoware_can_msgs::MicroBusCanSenderStatus &msg);//canステータス
	void callbackDistanceAngularCheck(const autoware_msgs::DifferenceToWaypointDistance &msg);
	void callbackDistanceAngularCheckNdt(const autoware_msgs::DifferenceToWaypointDistance &msg);
	void callbackDistanceAngularCheckEkf(const autoware_msgs::DifferenceToWaypointDistance &msg);
	void callbackDistanceAngularCheckGnss(const autoware_msgs::DifferenceToWaypointDistance &msg);
	void callbackConfig(const autoware_config_msgs::ConfigMicroBusCan &msg);
	void callbackLocalizerSelect(const std_msgs::Int32 &msg);//localizerの遷移状態 
	void callbackLocalizerMatchStat(const autoware_msgs::LocalizerMatchStat &msg);
	void callbackCanVelocityParam(const autoware_can_msgs::MicroBusCanVelocityParam &msg);
	void callbackStopperDistance(const autoware_msgs::StopperDistance &msg);
	void callbackWaypointParam(const autoware_msgs::WaypointParam &msg);
	void callbackImu(const sensor_msgs::Imu &msg);
	void callbackGnssPose(const geometry_msgs::PoseStamped &msg);
	void callbackGnssDeviation(const autoware_msgs::GnssStandardDeviation &msg);
	void callbackNdtStat(const autoware_msgs::NDTStat &msg);
	void callbackGnssStat(const std_msgs::UInt8 &msg);
	void callbackNdtStatString(const std_msgs::String &msg);
	void callbackStrokeRoutine(const std_msgs::String &msg);
	void callbackMobileyeCan(const can_msgs::Frame &msg);
	void callbackGnssTime(const autoware_system_msgs::Date &msg);
	void callbackLightColor(const autoware_msgs::TrafficLight &msg);
	void callbackSignalChangeTime(const std_msgs::Float64 &msg);
	void callbackPeriodSignalTakeover(const std_msgs::Bool &msg);
	void callbackAutomodeMileage(const std_msgs::Float64 &msg);
	void callbackVehicleCmd(const autoware_msgs::VehicleCmd &msg);
	void callbackCmdSelect(const std_msgs::Int32 &msg);
	void callbackLoadName(const autoware_msgs::WaypointsSerialNumLaunch &msg);
	void callbackBaseWaypoints(const autoware_msgs::LaneArray &msg);
	void callbackCurrentVelocity(const geometry_msgs::TwistStamped &msg);
	void callbackAcc(const std_msgs::Float64 &msg);
	void callbackAutoRouteLoopCount(const autoware_msgs::AutoRouteLoopCount::ConstPtr &msg);
	void callbackSafetyWaypoints(const autoware_msgs::Lane::ConstPtr &msg);
	void callbackSteerCorrectionWriteReturn(const std_msgs::String::ConstPtr &msg);
	void callbackPanelRead(const std_msgs::UInt16::ConstPtr &msg);
	void callbackPopupSignal(const autoware_msgs::InterfacePopupSignal::ConstPtr &msg);
	void callbackOncomingObs(const autoware_msgs::NearOncomingObs::ConstPtr &msg);
	void callbackSteerProofreadingMain(const autoware_can_msgs::SteerProofreading::ConstPtr &msg);
	void callbackSteerProofreadingBase(const autoware_can_msgs::SteerProofreading::ConstPtr &msg);
	void callbackFrontMobileye(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg);
	void callbackMobileyeCmdParam(const autoware_msgs::MobileyeCmdParam::ConstPtr &msg);
	void callbackTrackingType(const std_msgs::String::ConstPtr &msg);
	//void runWaypointsNode(std::string branch);

	autoware_can_msgs::MicroBusCan501 can501_;//マイコン応答ID501
	autoware_can_msgs::MicroBusCan502 can502_;//マイコン応答ID502
	autoware_can_msgs::MicroBusCan503 can503_;//マイコン応答ID503
	autoware_can_msgs::MicroBusCanSenderStatus can_status_;//canステータス
	autoware_msgs::DifferenceToWaypointDistance distance_angular_check_, distance_angular_check_ndt_, distance_angular_check_ekf_, distance_angular_check_gnss_;
	geometry_msgs::TwistStamped current_velocity_;//autowareからの現在の速度
	autoware_config_msgs::ConfigMicroBusCan config_;
	int localizer_select_;
	autoware_msgs::LocalizerMatchStat localizer_match_stat_;
	autoware_can_msgs::MicroBusCanVelocityParam can_velocity_param_;
	bool error_text_lock_;
	autoware_msgs::StopperDistance stopper_distance_;
	autoware_msgs::WaypointParam waypoint_param_;
	geometry_msgs::PoseStamped gnss_pose_;
	autoware_msgs::GnssStandardDeviation gnss_deviation_;
	autoware_system_msgs::Date gnss_time_;
	autoware_msgs::NDTStat ndt_stat_;
	unsigned char gnss_stat_;
	std::string ndt_stat_string_, stroke_routine_;
	sensor_msgs::Imu imu_;
	mobileye_560_660_msgs::AftermarketLane mobileye_lane_;
	std::string log_folder_;
	autoware_msgs::TrafficLight light_color_;
	double signal_change_time_;
	bool period_signal_takeover_;
	double automode_mileage_;
	autoware_msgs::VehicleCmd vehicle_cmd_;
	int cmd_select_;//ctrl_rawとtwist_rawをpublishしているノードの種類
	bool use_specified_cmd_;//固定指令速度を送信するか?
	double acc_;//加速度
	double steer_plus_sum_;
	int steer_plus_sum_count_;
	bool use_steer_plus_sum_;
	autoware_msgs::AutoRouteLoopCount auto_route_loop_count_;//自動経路切り替えの周回情報
	bool read_safety_waypoints_;
	ros::Duration localizer_time_diff_;
	PanelViewFlag panel_flag_front_;//車両前方パネル表示指令を管轄
	PanelViewFlag panel_flag_back_;//車両後方パネル表示指令を管轄
	uint8_t panel_view_front_;//車両前方パネル表示状態
	uint8_t panel_view_back_;//車両後方パネル表示状態
	uint16_t panel_signal_msg_;//パネルに送信した表示指令
	int prev_blinker_;//前回のウィンカー入力
	autoware_msgs::NearOncomingObs oncoming_obs_;//対向車が存在するか？
	int oncoming_stop_id_;//対向車停止線が存在するwaypointのID
	uint16_t rs232_steer_voltage_range_;//ステア自動校正用のサブ電圧レンジ
	uint16_t rs232_steer_voltage_straight_;//ステア自動校正用のサブ電圧レンジ範囲内での、直線走行時の電圧値
	std::string record_topic_list_;//rosbag recordするトピック一覧(カンマ区切り)
	bool rosbag_write_flag_;//rosbag記録中か？
	autoware_msgs::TransformMobileyeObstacle mobileye_front_car_;//mobileyeの前方車両情報
	autoware_msgs::MobileyeCmdParam mobileye_cmd_param_;//mobileye_trackerノードからpublishされる追跡用情報
	std::string tracking_type_;////前方車両追跡時の追従方法

	//タイマー
	ros::Time timer_error_lock_;

	//label color
	QPalette palette_drive_mode_ok_, palette_steer_mode_ok_;//autoモード表示テキストボックスのバックグラウンドカラーOK
	QPalette palette_drive_mode_error_, palette_steer_mode_error_;//autoモード表示テキストボックスのバックグラウンドカラーerror
	QPalette palette_drive_clutch_connect_, palette_drive_clutch_cut_;//ドライブクラッチのテキストボックスパレット
	QPalette palette_steer_clutch_connect_, palette_steer_clutch_cut_;//ハンドルクラッチのテキストボックスパレット
	QPalette palette_distance_angular_ok_, palette_distance_angular_error_;//経路との距離と角度チェックのテキストボックスパレット
	QPalette palette_localizer_select_ok_, palette_localizer_select_error_;//localizerの遷移状態のテキストボックスパレット
	QPalette palette_gnss_deviation_ok_, palette_gnss_deviation_error_;//gnssの誤差
	QPalette palette_score_ok_, palette_score_error_;
	QPalette palette_current_localizer_, palette_lb_normal_, palette_lb_localize_;
	QPalette palette_signal_text_green_, palette_signal_text_red_, palette_signal_text_unknown_, palette_period_signal_takeover_;//信号関連
	QPalette palette_stop_line_non_, palette_stop_line_middle_, palette_stop_line_stop_;
	QPalette palette_auto_check_ok_, palette_auto_check_error_;
	QPalette palette_logbt_on_, palette_logbt_off_;

	std::string gnss_time_str();
	void error_view(std::string error_message);
	void specified_speed_add(double add);
	void specified_deg_add(double add);
	void dialog_driving_adjustment_show();
	void rosbag_write();
	void rosbag_stop();

	double signal_red_green_time_, signal_green_yellow_time_, signal_yellow_red_time_, signal_red_green_time2_;

	std::vector<QString> panel_front_text_ = {
		"自動運転中",
		"お先にどうぞ",
		"埼玉工業大学",
		"右折します",
		"対向車を 待ちますか？",
		"まもなく赤 減速します",
		"法令速度遵守",
	};

	std::vector<QString> panel_back_text_ = {
		"自動運転中",
		"お先にどうぞ",
		"埼玉工業大学",
		"右折します",
		"対向車を 待ちますか？",
		"まもなく赤 減速します",
		"法令速度遵守",
	};

	std::vector<QString> select_launch_text_ = {"渋沢巡回", "岡部便", "茂木実証","埼玉工業大学ロータリー", "幕張実証"};
	std::vector<std::string> select_launch_file_ = {"00A", "daigaku_go", "motegi_1_mitinoeki_motegieki","daigaku_rotari", "makuhari_1_iron_zozo"};
	/*std::vector<std::vector<std::string>> select_launch_ = {
		{"1-00A","1-01A","1-02A","1-03A","1-04A","1-05A","1-06B","1-02A","1-03A","1-04A","1-05A","1-06A","1-07A","1-08A","1-09A","1-10A",
		 "2-00A","2-01A","2-02A","2-03A","2-04A","2-05A","2-06A","2-07A","2-08A","2-09A","2-10A",
		 "3-00A","3-01A","3-02A","3-03A","3-04A","3-05A","3-06B","3-02A","3-03A","3-04A","3-05A","3-06A","3-07A","3-08A","3-09A","3-10A",
		 "4-00A","4-01A","4-02A","4-03A","4-04A","4-05A","4-06B","4-02A","4-03A","4-04A","4-05A","4-06A","4-07A","4-08A","4-09A","4-10A",
		 "5-00A","5-01A","5-02A","5-03A","5-04A","5-05A","5-06B","5-02A","5-03A","5-04A","5-05A","5-06A","5-07A","5-08A","5-09A","5-10A",
		 "6-00A","6-01A","6-02A","6-03A","6-04A","6-05A","6-06B","6-02A","6-03A","6-04A","6-05A","6-06A","6-07A","6-08A","6-09A","6-10A",
		 "7-00A","7-01A","7-02A","7-03A","7-04A","7-05A","7-06A","7-07A","7-08A","7-09A","7-10A"},
		{"aa"},
		{"bbb"}
	};*/

	//wrote by minamidani
	std::vector<std::vector<std::string>> select_launch_ = {
		{"1-00A","1-01A","1-02A","1-03A","1-04A","1-05A","1-06B","1-02A","1-03A","1-04A","1-05A","1-06A","1-07A","1-08A","1-09A","1-10A",
		 "2-00A","2-01A","2-02A","2-03A","2-04A","2-05A","2-06A","2-07A","2-08A","2-09A","2-10A",
		 "3-00A","3-01A","3-02A","3-03A","3-04A","3-05A","3-06B","3-02A","3-03A","3-04A","3-05A","3-06A","3-07A","3-08A","3-09A","3-10A",
		 "4-00A","4-01A","4-02A","4-03A","4-04A","4-05A","4-06B","4-02A","4-03A","4-04A","4-05A","4-06A","4-07A","4-08A","4-09A","4-10A",
		 "5-00A","5-01A","5-02A","5-03A","5-04A","5-05A","5-06A","5-07A","5-08A","5-09A","5-10A",
		 "6-00A","6-01A","6-02A","6-03A","6-04A","6-05A","6-06B","6-02A","6-03A","6-04A","6-05A","6-06A","6-07A","6-08A","6-09A","6-10A",
		 "7-00A","7-01A","7-02A","7-03A","7-04A","7-05A","7-06B","7-02A","7-03A","7-04A","7-05A","7-06A","7-07A","7-08A","7-09A","7-10A"},
		{"1-daigaku_go","1-daigaku_return"},
		{"1-bbb"},
		{"1-daigaku_rotari"},
		{"1-makuhari_1_iron_zozo", "1-makuhari_2_zozo_yuraku","1-makuhari_3_yuraku_iron",
		 "2-makuhari_1_iron_zozo", "2-makuhari_2_zozo_yuraku","2-makuhari_3_yuraku_iron",
		 "3-makuhari_1_iron_zozo", "3-makuhari_2_zozo_yuraku","3-makuhari_3_yuraku_iron",
		 "4-makuhari_1_iron_zozo", "4-makuhari_2_zozo_yuraku","4-makuhari_3_yuraku_iron",
		 "5-makuhari_1_iron_zozo", "5-makuhari_2_zozo_yuraku","5-makuhari_3_yuraku_iron"}
	};

	std::vector<LaneName> lane_name_ = {
		{"00A", "仲町バス発着所", "大河ドラマ館"},
		{"01A", "大河ドラマ館", "北部運動公園"},
		{"02A", "北部運動公園", "尾高惇忠生家"},
		{"03A", "尾高惇忠生家", "渋沢栄一記念館"},
		{"04A", "渋沢栄一記念館", "中の家"},
		{"05A", "中の家", "道の駅おかべ"},
		{"06A", "道の駅おかべ", "岡部公会堂"},
		{"06B", "道の駅おかべ", "北部運動公園"},
		{"07A", "岡部公会堂", "西常夜灯"},
		{"08A", "西常夜灯", "七ツ梅"},
		{"09A", "七ツ梅", "深谷駅北口"},
		{"10A", "深谷駅北口", "仲町バス発着所"},
		{"daigaku_go", "埼玉工業大学", "岡部駅ロータリー"},
		{"daigaku_return", "岡部駅ロータリー", "埼玉工業大学"},
		{"daigaku_rotari", "先端科学研究所前", "先端科学研究所前"},
		{"makuhari_1_iron_zozo", "イオンモール", "ZOZOマリンスタジアム"},
		{"makuhari_2_zozo_yuraku", "ZOZOマリンスタジアム", "湯楽の里"},
		{"makuhari_3_yuraku_iron", "湯楽の里", "イオンモール"}
	};

	Dialog_driving_adjustment* dialog_driving_adjustment_;
    Dialog_popup_signal* dialog_popup_signal_;
private slots:
	void publish_emergency_clear();
	void publish_Dmode_manual();
	void publish_Dmode_program();
	void publish_Smode_manual();
	void publish_Smode_program();
	void publish_Dmode_velocity();
	void publish_Dmode_stroke();
	void publish_Dmode_input_direct();
	void publish_Dmode_input_auto();
	void publish_Smode_input_direct();
	void publish_Smode_input_auto();
	void publish_drive_clutch_connect();
	void publish_drive_clutch_cut();
	void publish_steer_clutch_connect();
	void publish_steer_clutch_cut();
	void publish_blinker_right();
	void publish_blinker_left();
	void publish_blinker_stop();
	void publish_use_safety_localizer();
	void publish_use_error_check();
	void publish_log_write();
	void publish_log_stop();
	void publish_specified_speed();
	void publish_specified_speed_stop();
	void click_error_text_reset();
	void click_signal_time();
	void click_signal_time_clear();
	void click_log_folder();
	void click_load_nextA();
	void click_load_backA();
	void click_load_nextB();
	void click_load_backB();
	void click_auto_change_loop_next();
	void click_auto_change_loop_back();
	void click_specified_speed_plus1();
	void click_specified_speed_minus1();
	void click_specified_speed_plus5();
	void click_specified_speed_minus5();
	void click_specified_deg_plus1();
	void click_specified_deg_minus1();
	void click_specified_deg_plus5();
	void click_specified_deg_minus5();
	void click_specified_deg_plus_any();
	void click_specified_deg_minus_any();
	void click_steer_plus();
	void click_rviz_restart();
	void click_steer_plus_ave();
	void click_steer_plus_ave_stop();
	void click_steer_P10();
	void click_steer_M10();
	void click_arena_gain_send();
	void click_auto_shutdown();
	void click_call_dialog_driving_adjustment();
	void click_steer_proofreading_main();
	void click_steer_proofreading_base();
	void click_rosbag();
	void click_track_excess_acc();
	void click_track_excess_stop();
	void click_yure();
	void check_use_auto_chenge(bool flag);
	void check_use_auto_log(bool flag);
	void slide_specified_speed(int val);
	void slide_specified_deg(int val);
	void car_target_deceleration_change(double val);
	void currentItemchange_list_read_launch(QListWidgetItem* new_item,QListWidgetItem* prev_item);
};

#endif // MAINWINDOW_H
