#ifndef DIALOG_ROSBAG_H
#define DIALOG_ROSBAG_H

#include <QDialog>
#include <QStandardItemModel>

namespace Ui {
class Dialog_rosbag;
}

class Dialog_rosbag : public QDialog
{
	Q_OBJECT

public:
	explicit Dialog_rosbag(QWidget *parent = 0);
	~Dialog_rosbag();

	//void listClear();
	void setSubscribeTopicName(const std::string name);
	void setRecordTopicName(const std::string name);
	std::string recordTopicList() const;
	bool ok() const;
protected:
	void showEvent(QShowEvent *e);
private:
	Ui::Dialog_rosbag *ui;

	bool ok_flag_;
private slots:
	void clickRecordAdd();
	void clickSubscribeDelete();
	void click_ok();
	void click_cancel();
};

#endif // DIALOG_ROSBAG_H
