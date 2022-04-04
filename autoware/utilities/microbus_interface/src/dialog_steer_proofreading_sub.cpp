#include "dialog_steer_proofreading_sub.h"
#include "ui_dialog_steer_proofreading_sub.h"

dialog_steer_proofreading_sub::dialog_steer_proofreading_sub(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::dialog_steer_proofreading_sub),
	straight_sub_voltage_(-1)
{
	ui->setupUi(this);

	connect(ui->bt_ok, SIGNAL(clicked()), this, SLOT(click_ok()));
	connect(ui->bt_cancel, SIGNAL(clicked()), this, SLOT(click_cancel()));
}

dialog_steer_proofreading_sub::~dialog_steer_proofreading_sub()
{
	delete ui;
}

void dialog_steer_proofreading_sub::setText(const std::string text)
{
	ui->lb_text->setText(text.c_str());
}

int dialog_steer_proofreading_sub::straightSubVoltage()
{
	return straight_sub_voltage_;
}

void dialog_steer_proofreading_sub::end_form(const bool ok)
{
	if(ok == true)
	{
		straight_sub_voltage_ = 0;
		try
		{
			int val = std::stoi(ui->tx_straight_sub_voltage->text().toStdString());
			if(val >= 0 && val <= 2048)
			{
				straight_sub_voltage_ = val;
				close();
			}
		}
		catch(const std::exception& e)
		{
		}
	}
	else
	{
		close();
	}
}

void dialog_steer_proofreading_sub::click_ok()
{
	end_form(true);
}

void dialog_steer_proofreading_sub::click_cancel()
{
	end_form(false);
}