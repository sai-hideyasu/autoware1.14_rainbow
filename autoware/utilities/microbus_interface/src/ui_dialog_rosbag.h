/********************************************************************************
** Form generated from reading UI file 'dialog_rosbag.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_ROSBAG_H
#define UI_DIALOG_ROSBAG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QListView>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Dialog_rosbag
{
public:
    QListView *li_recording_topic;
    QListView *li_subscribe_topic;
    QPushButton *bt_ok;
    QPushButton *bt_cancel;
    QPushButton *bt_subscribe_delete;
    QPushButton *bt_record_add;

    void setupUi(QDialog *Dialog_rosbag)
    {
        if (Dialog_rosbag->objectName().isEmpty())
            Dialog_rosbag->setObjectName(QStringLiteral("Dialog_rosbag"));
        Dialog_rosbag->resize(835, 700);
        li_recording_topic = new QListView(Dialog_rosbag);
        li_recording_topic->setObjectName(QStringLiteral("li_recording_topic"));
        li_recording_topic->setGeometry(QRect(10, 10, 381, 521));
        QFont font;
        font.setPointSize(15);
        li_recording_topic->setFont(font);
        li_recording_topic->setEditTriggers(QAbstractItemView::NoEditTriggers);
        li_subscribe_topic = new QListView(Dialog_rosbag);
        li_subscribe_topic->setObjectName(QStringLiteral("li_subscribe_topic"));
        li_subscribe_topic->setGeometry(QRect(440, 10, 381, 521));
        li_subscribe_topic->setFont(font);
        li_subscribe_topic->setEditTriggers(QAbstractItemView::NoEditTriggers);
        bt_ok = new QPushButton(Dialog_rosbag);
        bt_ok->setObjectName(QStringLiteral("bt_ok"));
        bt_ok->setGeometry(QRect(150, 620, 241, 81));
        QFont font1;
        font1.setPointSize(30);
        bt_ok->setFont(font1);
        bt_cancel = new QPushButton(Dialog_rosbag);
        bt_cancel->setObjectName(QStringLiteral("bt_cancel"));
        bt_cancel->setGeometry(QRect(440, 620, 241, 81));
        bt_cancel->setFont(font1);
        bt_subscribe_delete = new QPushButton(Dialog_rosbag);
        bt_subscribe_delete->setObjectName(QStringLiteral("bt_subscribe_delete"));
        bt_subscribe_delete->setGeometry(QRect(90, 530, 241, 61));
        bt_subscribe_delete->setFont(font1);
        bt_record_add = new QPushButton(Dialog_rosbag);
        bt_record_add->setObjectName(QStringLiteral("bt_record_add"));
        bt_record_add->setGeometry(QRect(500, 530, 241, 61));
        bt_record_add->setFont(font1);

        retranslateUi(Dialog_rosbag);

        QMetaObject::connectSlotsByName(Dialog_rosbag);
    } // setupUi

    void retranslateUi(QDialog *Dialog_rosbag)
    {
        Dialog_rosbag->setWindowTitle(QApplication::translate("Dialog_rosbag", "Dialog", Q_NULLPTR));
        bt_ok->setText(QApplication::translate("Dialog_rosbag", "OK", Q_NULLPTR));
        bt_cancel->setText(QApplication::translate("Dialog_rosbag", "\343\202\255\343\203\243\343\203\263\343\202\273\343\203\253", Q_NULLPTR));
        bt_subscribe_delete->setText(QApplication::translate("Dialog_rosbag", "\345\211\212\351\231\244", Q_NULLPTR));
        bt_record_add->setText(QApplication::translate("Dialog_rosbag", "\350\277\275\345\212\240", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Dialog_rosbag: public Ui_Dialog_rosbag {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_ROSBAG_H
