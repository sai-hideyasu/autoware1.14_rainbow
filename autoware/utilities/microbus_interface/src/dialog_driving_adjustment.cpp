#include "mainwindow.h"
#include "dialog_driving_adjustment.h"
#include "ui_dialog_driving_adjustment.h"

Dialog_driving_adjustment::Dialog_driving_adjustment(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::Dialog_driving_adjustment),
	steer_plus_sub_(0),
	din0_(false),
	din1_(false),
	din2_(false),
	din3_(false),
	din_timer_id_(0),
	show_timer_id_(0),
	first_show_flag_(false),
	waypoint_base_corr_(WAYPOINT_CORR_NO_READ),
	waypoint_sub_corr_(WAYPOINT_CORR_NO_READ)
{
	ui->setupUi(this);

	connect(ui->bt_end, SIGNAL(clicked()), this, SLOT(click_end()));
	connect(ui->bt_left, SIGNAL(clicked()), this, SLOT(click_left()));
	connect(ui->bt_right, SIGNAL(clicked()), this, SLOT(click_right()));
	connect(ui->bt_cancel, SIGNAL(clicked()), this, SLOT(click_cancel()));

	ui->li_steer_plus_param_->setText(std::to_string(steer_plus_sub_).c_str());
}

Dialog_driving_adjustment::~Dialog_driving_adjustment()
{
	delete ui;
}

void Dialog_driving_adjustment::showEvent(QShowEvent *e)
{
	first_show_flag_ = true;
	din_timer_id_ = startTimer(TIMER_START_DIN);
	show_timer_id_ = startTimer(TIMER_SHOW);
}

void Dialog_driving_adjustment::closeEvent(QCloseEvent *e)
{
	killTimer(din_timer_id_);
}

//canからのボタン指令を受け取る
void Dialog_driving_adjustment::setDin(const bool setdin0, const bool setdin1, const bool setdin2, const bool setdin3)
{
	if(isHidden())
	{
		if(setdin0 && setdin1)
		{
			show();
			/*first_show_flag_ = true;
			din_timer_id_ = startTimer(TIMER_START_DIN);
			show_timer_id_ = startTimer(TIMER_SHOW);*/
		}
	}
	din0_ = setdin0;//確定
	din1_ = setdin1;//キャンセル
	din2_ = setdin2;//左調整
	din3_ = setdin3;//右調整
}

//waypoint情報からsteer調整項目を取得する
void Dialog_driving_adjustment::setWaypointCorrection(const int16_t base_corr, const int16_t sub_corr)
{
	waypoint_base_corr_ = base_corr;//ベース調整値　常に同じ値
	waypoint_sub_corr_ = sub_corr;//サブ調整値　waypoint毎に変わる調整項
	ui->li_base->setText(QString::fromStdString(std::to_string(waypoint_base_corr_)));
	ui->li_waypoint_corr->setText(QString::fromStdString(std::to_string(waypoint_sub_corr_)));
	ui->li_all_corr->setText(QString::fromStdString(std::to_string(waypoint_sub_corr_ + steer_plus_sub_)));//現在のサブ調整値とwaypointのサブ調整値の和
}

//
void Dialog_driving_adjustment::leftArrow()
{
	steer_plus_sub_ -= 10;
	ui->li_steer_plus_param_->setText(std::to_string(steer_plus_sub_).c_str());
	if(waypoint_base_corr_ != WAYPOINT_CORR_NO_READ && waypoint_sub_corr_ != WAYPOINT_CORR_NO_READ)
		ui->li_all_corr->setText(QString::fromStdString(std::to_string(waypoint_sub_corr_ + steer_plus_sub_)));
}

void Dialog_driving_adjustment::rightArrow()
{
	steer_plus_sub_ += 10;
	ui->li_steer_plus_param_->setText(std::to_string(steer_plus_sub_).c_str());
	if(waypoint_base_corr_ != WAYPOINT_CORR_NO_READ && waypoint_sub_corr_ != WAYPOINT_CORR_NO_READ)
		ui->li_all_corr->setText(QString::fromStdString(std::to_string(waypoint_sub_corr_ + steer_plus_sub_)));
}

void Dialog_driving_adjustment::endFrom(const bool per)
{
	close();
	valueClear();
	MainWindow *main_window = (MainWindow*)this->parent();
	main_window->steer_sub_interface_correction_write_end(per);
}

void Dialog_driving_adjustment::timerEvent(QTimerEvent * e)
{
	if(e->timerId() == show_timer_id_)//ダイアログ表示後に一定時間がたったらcanボタンからの入力を受け付ける
	{
		first_show_flag_ = false;
		killTimer(show_timer_id_);
	}
	else if(e->timerId() == din_timer_id_)
	{
		if(first_show_flag_ == true) return;//立ち上げ直後は不用意なcan入力を回避する

		if(din0_)//確定
		{
			endFrom(true);
			//killTimer(din_timer_id_);
		}
		else if(din1_)//キャンセル
		{
			endFrom(false);
			//killTimer(din_timer_id_);
		}
		else if(din2_) leftArrow();//左調整
		else if(din3_) rightArrow();//右調整
	}
}

void Dialog_driving_adjustment::valueClear()
{
	steer_plus_sub_ = 0;
	ui->li_steer_plus_param_->setText(std::to_string(steer_plus_sub_).c_str());
}

int16_t Dialog_driving_adjustment::getSubSteer()
{
	return steer_plus_sub_;
}

void Dialog_driving_adjustment::click_end()
{
	endFrom(true);
}

void Dialog_driving_adjustment::click_cancel()
{
	endFrom(false);
}

void Dialog_driving_adjustment::click_left()
{
	leftArrow();
}

void Dialog_driving_adjustment::click_right()
{
	rightArrow();
}