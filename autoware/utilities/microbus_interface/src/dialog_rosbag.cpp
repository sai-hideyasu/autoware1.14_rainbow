#include "dialog_rosbag.h"
#include "ui_dialog_rosbag.h"
#include <QMessageBox>
#include <sstream>

Dialog_rosbag::Dialog_rosbag(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::Dialog_rosbag),
	ok_flag_(false)
{
	ui->setupUi(this);

	//各リストビュー表示用のモデルを設定
	QStandardItemModel *record_model = new QStandardItemModel();
	ui->li_recording_topic->setModel((QAbstractItemModel*)record_model);
	QStandardItemModel *subscribe_model = new QStandardItemModel();
	//name_model->sort(, Qt::AscendingOrder);
	ui->li_subscribe_topic->setModel((QAbstractItemModel*)subscribe_model);

	connect(ui->bt_record_add, SIGNAL(clicked()), this, SLOT(clickRecordAdd()));
	connect(ui->bt_subscribe_delete, SIGNAL(clicked()), this, SLOT(clickSubscribeDelete()));
	connect(ui->bt_ok, SIGNAL(clicked()), this, SLOT(click_ok()));
	connect(ui->bt_cancel, SIGNAL(clicked()), this, SLOT(click_cancel()));
}

Dialog_rosbag::~Dialog_rosbag()
{
	delete ui;
}

/*void Dialog_rosbag::listClear()
{
	QStandardItemModel *subscribe_model = static_cast<QStandardItemModel*>(ui->li_subscribe_topic->model());
	subscribe_model->clear();
	QStandardItemModel *record_model = static_cast<QStandardItemModel*>(ui->li_recording_topic->model());
	record_model->clear();
}*/

void Dialog_rosbag::setSubscribeTopicName(const std::string name)
{
	QStandardItemModel *subscribe_model = static_cast<QStandardItemModel*>(ui->li_subscribe_topic->model());

	//トピック名を追加
	QStandardItem *subscribe_item = new QStandardItem();
	subscribe_item->setText(QString::fromStdString(name));
	subscribe_item->setEditable(false);
	subscribe_model->appendRow(subscribe_item);
}

void Dialog_rosbag::setRecordTopicName(const std::string name)
{
	QStandardItemModel *record_model = static_cast<QStandardItemModel*>(ui->li_recording_topic->model());

	//トピック名を追加
	QStandardItem *record_item = new QStandardItem();
	record_item->setText(QString::fromStdString(name));
	record_item->setEditable(false);
	record_model->appendRow(record_item);
}

void Dialog_rosbag::showEvent(QShowEvent *e)
{
	ok_flag_ = false;
	QStandardItemModel *subscribe_model = static_cast<QStandardItemModel*>(ui->li_subscribe_topic->model());
	subscribe_model->sort(0, Qt::AscendingOrder);
	QStandardItemModel *record_model = static_cast<QStandardItemModel*>(ui->li_recording_topic->model());
	record_model->sort(0, Qt::AscendingOrder);
}

void Dialog_rosbag::clickRecordAdd()
{
	QStandardItemModel *record_model = static_cast<QStandardItemModel*>(ui->li_recording_topic->model());
	const QStandardItemModel *subscribe_model = static_cast<QStandardItemModel*>(ui->li_subscribe_topic->model());

	const QModelIndexList subscribe_select_indexs = ui->li_subscribe_topic->selectionModel()->selectedIndexes();

	for(const QModelIndex subscribe_index : subscribe_select_indexs)
	{
		const QStandardItem* subscribe_item = subscribe_model->itemFromIndex(subscribe_index);

		bool new_item = true;
		for(int row=0; row<record_model->rowCount(); row++)
		{
			QStandardItem* record_item = record_model->item(row, 0);
			if(record_item->text() == subscribe_item->text())
			{
				new_item = false;
				break;
			}
		}

		if(new_item == true)
		{
			QStandardItem* new_record_item = new QStandardItem(subscribe_item->text());
			new_record_item->setEditable(false);
			record_model->appendRow(new_record_item);
		}
	}

	record_model->sort(0, Qt::AscendingOrder);
}

void Dialog_rosbag::clickSubscribeDelete()
{
	QStandardItemModel *record_model = static_cast<QStandardItemModel*>(ui->li_recording_topic->model());

	const QModelIndexList indexs = ui->li_recording_topic->selectionModel()->selectedIndexes();

	for(int cou=indexs.size()-1; cou>=0; cou--)
	{
		record_model->removeRow(indexs[cou].row());
	}
}

bool Dialog_rosbag::ok() const
{
	return ok_flag_;
}

std::string Dialog_rosbag::recordTopicList() const
{
	std::stringstream ss;
	QStandardItemModel *record_model = static_cast<QStandardItemModel*>(ui->li_recording_topic->model());
	for(int i=0; i<record_model->rowCount(); i++)
	{
		QStandardItem* record_item = record_model->item(i, 0);
		ss << record_item->text().toStdString();
		if(i != record_model->rowCount()-1) ss << ',';
	}
	return ss.str();
}

void Dialog_rosbag::click_ok()
{
	ok_flag_ = true;
	close();
}

void Dialog_rosbag::click_cancel()
{
	ok_flag_ = false;
	close();
}