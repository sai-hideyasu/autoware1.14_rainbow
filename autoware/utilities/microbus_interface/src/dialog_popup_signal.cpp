#include "mainwindow.h"
#include "dialog_popup_signal.h"
#include "ui_dialog_popup_signal.h"

std::string replaceOtherStr(const std::string &replacedStr, const char* from, const char* to) 
{
    std::string str = replacedStr;
    for(;;)
    {
        int pos = str.find(from);
        //std::cout << pos << std::endl;
        if(pos < 0) break;
        str = str.replace(pos, 1, to);
    }

    return str;
}

Dialog_popup_signal::Dialog_popup_signal(bool use_accel_intervention, QWidget *parent) :
	QDialog(parent),
    ui(new Ui::Dialog_popup_signal),
	signal_(0),
	use_accel_intervention_(use_accel_intervention),
	on_accel_intervention_(false),
	din0_(false),
	din1_(false),
	din2_(false),
	din3_(false),
	accel_intervention_(false),
	din_timer_id_(0),
	process_flag_(false),
	way_id_(0)
{
	ui->setupUi(this);

	connect(ui->bt_end, SIGNAL(clicked()), this, SLOT(click_end()));
	connect(ui->bt_cancel, SIGNAL(clicked()), this, SLOT(click_cancel()));
}

Dialog_popup_signal::~Dialog_popup_signal()
{
	delete ui;
}

void Dialog_popup_signal::showEvent(QShowEvent *e)
{
	process_flag_ = true;
	din_timer_id_ = startTimer(TIMER_START_DIN);
}

void Dialog_popup_signal::timerEvent(QTimerEvent * e)
{
	if(e->timerId() == din_timer_id_)
	{
		if(din0_)//確定
		{
			endFrom(true);
		}
		else if(use_accel_intervention_ && on_accel_intervention_ && accel_intervention_)//アクセル介入
		{
			endFrom(true);
		}
		else if(din1_)//キャンセル
		{
			endFrom(false);
		}
	}
	else if(e->timerId() == end_timer_id_ )
	{
		process_flag_ = false;
		killTimer(end_timer_id_);
	}
}

bool Dialog_popup_signal::getProcessFlag() const
{
	return process_flag_;
}

void Dialog_popup_signal::setWaypointID(const uint32_t id)
{
	way_id_ = id;
}

void Dialog_popup_signal::setPopupSignal(const uint8_t signal, const std::string str, const bool on_acc_intervention)
{
	signal_ = signal;	
	std::string str_rep = replaceOtherStr(str, "　", "\n");
	str_rep = replaceOtherStr(str_rep, " ", "\n");
	ui->lb_text->setText(str_rep.c_str());
	on_accel_intervention_ = on_acc_intervention;
}

//canからのボタン指令を受け取る
void Dialog_popup_signal::setDin(const bool setdin0, const bool setdin1, const bool setdin2, const bool setdin3,
	const bool accel_intervention)
{
	din0_ = setdin0;//確定
	din1_ = setdin1;//キャンセル
	din2_ = setdin2;//左調整
	din3_ = setdin3;//右調整
	accel_intervention_ = accel_intervention;
}

void Dialog_popup_signal::endFrom(const bool ok)
{
	close();
	MainWindow *main_window = (MainWindow*)this->parent();
    main_window->popup_signal_interface_ret(ok, signal_, way_id_);
	end_timer_id_ = startTimer(TIMER_END);
	on_accel_intervention_ = false;
}

void Dialog_popup_signal::click_end()
{
	endFrom(true);
}

void Dialog_popup_signal::click_cancel()
{
	endFrom(false);
}

void Dialog_popup_signal::blinkerCessation()
{
	if(signal_ == autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_LEFT
		|| signal_ == autoware_msgs::InterfacePopupSignal::SIGNAL_BLINKER_RIGHT)
	{
		close();
	}
}