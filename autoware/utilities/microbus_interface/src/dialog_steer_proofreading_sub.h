#ifndef DIALOG_STEER_PROOFREADING_SUB_H
#define DIALOG_STEER_PROOFREADING_SUB_H

#include <QDialog>

namespace Ui {
class dialog_steer_proofreading_sub;
}

class dialog_steer_proofreading_sub : public QDialog
{
	Q_OBJECT

public:
	explicit dialog_steer_proofreading_sub(QWidget *parent = 0);
	~dialog_steer_proofreading_sub();

	void setText(const std::string text);
	int straightSubVoltage();
private:
	Ui::dialog_steer_proofreading_sub *ui;

	int straight_sub_voltage_;
	void end_form(const bool ok);
private slots:
	void click_ok();
	void click_cancel();
};

#endif // DIALOG_STEER_PROOFREADING_SUB_H
