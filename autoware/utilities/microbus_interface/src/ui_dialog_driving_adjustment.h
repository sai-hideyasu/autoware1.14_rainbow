/********************************************************************************
** Form generated from reading UI file 'dialog_driving_adjustment.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_DRIVING_ADJUSTMENT_H
#define UI_DIALOG_DRIVING_ADJUSTMENT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Dialog_driving_adjustment
{
public:
    QPushButton *bt_end;
    QPushButton *bt_cancel;
    QGroupBox *gb_steer_plus_param;
    QLineEdit *li_steer_plus_param_;
    QPushButton *bt_right;
    QPushButton *bt_left;
    QLabel *lb_base;
    QLabel *lb_waypoint_corr;
    QLineEdit *li_base;
    QLineEdit *li_waypoint_corr;
    QLineEdit *li_all_corr;
    QLabel *lb_all_corr;

    void setupUi(QDialog *Dialog_driving_adjustment)
    {
        if (Dialog_driving_adjustment->objectName().isEmpty())
            Dialog_driving_adjustment->setObjectName(QStringLiteral("Dialog_driving_adjustment"));
        Dialog_driving_adjustment->resize(460, 490);
        bt_end = new QPushButton(Dialog_driving_adjustment);
        bt_end->setObjectName(QStringLiteral("bt_end"));
        bt_end->setGeometry(QRect(10, 400, 201, 81));
        QFont font;
        font.setPointSize(30);
        bt_end->setFont(font);
        bt_cancel = new QPushButton(Dialog_driving_adjustment);
        bt_cancel->setObjectName(QStringLiteral("bt_cancel"));
        bt_cancel->setGeometry(QRect(250, 400, 201, 81));
        bt_cancel->setFont(font);
        gb_steer_plus_param = new QGroupBox(Dialog_driving_adjustment);
        gb_steer_plus_param->setObjectName(QStringLiteral("gb_steer_plus_param"));
        gb_steer_plus_param->setGeometry(QRect(10, 120, 441, 141));
        QFont font1;
        font1.setPointSize(24);
        gb_steer_plus_param->setFont(font1);
        gb_steer_plus_param->setAlignment(Qt::AlignCenter);
        li_steer_plus_param_ = new QLineEdit(gb_steer_plus_param);
        li_steer_plus_param_->setObjectName(QStringLiteral("li_steer_plus_param_"));
        li_steer_plus_param_->setGeometry(QRect(110, 50, 221, 81));
        QFont font2;
        font2.setPointSize(34);
        li_steer_plus_param_->setFont(font2);
        li_steer_plus_param_->setAlignment(Qt::AlignCenter);
        li_steer_plus_param_->setReadOnly(true);
        bt_right = new QPushButton(gb_steer_plus_param);
        bt_right->setObjectName(QStringLiteral("bt_right"));
        bt_right->setGeometry(QRect(340, 50, 91, 81));
        bt_right->setFont(font);
        bt_right->setAutoRepeat(true);
        bt_right->setAutoRepeatDelay(0);
        bt_right->setAutoRepeatInterval(100);
        bt_left = new QPushButton(gb_steer_plus_param);
        bt_left->setObjectName(QStringLiteral("bt_left"));
        bt_left->setGeometry(QRect(10, 50, 91, 81));
        bt_left->setFont(font);
        bt_left->setAutoRepeat(true);
        bt_left->setAutoRepeatDelay(0);
        bt_left->setAutoRepeatInterval(100);
        lb_base = new QLabel(Dialog_driving_adjustment);
        lb_base->setObjectName(QStringLiteral("lb_base"));
        lb_base->setGeometry(QRect(10, 0, 201, 31));
        QFont font3;
        font3.setPointSize(20);
        lb_base->setFont(font3);
        lb_base->setAlignment(Qt::AlignCenter);
        lb_waypoint_corr = new QLabel(Dialog_driving_adjustment);
        lb_waypoint_corr->setObjectName(QStringLiteral("lb_waypoint_corr"));
        lb_waypoint_corr->setGeometry(QRect(250, 0, 201, 31));
        lb_waypoint_corr->setFont(font3);
        lb_waypoint_corr->setAlignment(Qt::AlignCenter);
        li_base = new QLineEdit(Dialog_driving_adjustment);
        li_base->setObjectName(QStringLiteral("li_base"));
        li_base->setGeometry(QRect(10, 40, 201, 81));
        li_base->setFont(font2);
        li_base->setAlignment(Qt::AlignCenter);
        li_base->setReadOnly(true);
        li_waypoint_corr = new QLineEdit(Dialog_driving_adjustment);
        li_waypoint_corr->setObjectName(QStringLiteral("li_waypoint_corr"));
        li_waypoint_corr->setGeometry(QRect(250, 40, 201, 81));
        li_waypoint_corr->setFont(font2);
        li_waypoint_corr->setAlignment(Qt::AlignCenter);
        li_waypoint_corr->setReadOnly(true);
        li_all_corr = new QLineEdit(Dialog_driving_adjustment);
        li_all_corr->setObjectName(QStringLiteral("li_all_corr"));
        li_all_corr->setGeometry(QRect(120, 310, 221, 81));
        li_all_corr->setFont(font2);
        li_all_corr->setAlignment(Qt::AlignCenter);
        li_all_corr->setReadOnly(true);
        lb_all_corr = new QLabel(Dialog_driving_adjustment);
        lb_all_corr->setObjectName(QStringLiteral("lb_all_corr"));
        lb_all_corr->setGeometry(QRect(10, 270, 441, 31));
        lb_all_corr->setFont(font3);
        lb_all_corr->setAlignment(Qt::AlignCenter);

        retranslateUi(Dialog_driving_adjustment);

        QMetaObject::connectSlotsByName(Dialog_driving_adjustment);
    } // setupUi

    void retranslateUi(QDialog *Dialog_driving_adjustment)
    {
        Dialog_driving_adjustment->setWindowTitle(QApplication::translate("Dialog_driving_adjustment", "Dialog", Q_NULLPTR));
        bt_end->setText(QApplication::translate("Dialog_driving_adjustment", "OK", Q_NULLPTR));
        bt_cancel->setText(QApplication::translate("Dialog_driving_adjustment", "\343\202\255\343\203\243\343\203\263\343\202\273\343\203\253", Q_NULLPTR));
        gb_steer_plus_param->setTitle(QApplication::translate("Dialog_driving_adjustment", "\347\217\276\345\234\250\343\201\256\350\252\277\346\225\264\345\200\244", Q_NULLPTR));
        li_steer_plus_param_->setText(QApplication::translate("Dialog_driving_adjustment", "0", Q_NULLPTR));
        bt_right->setText(QApplication::translate("Dialog_driving_adjustment", "\342\206\222", Q_NULLPTR));
        bt_left->setText(QApplication::translate("Dialog_driving_adjustment", "\342\206\220", Q_NULLPTR));
        lb_base->setText(QApplication::translate("Dialog_driving_adjustment", "\343\203\231\343\203\274\343\202\271\345\200\244", Q_NULLPTR));
        lb_waypoint_corr->setText(QApplication::translate("Dialog_driving_adjustment", "\347\265\214\350\267\257\343\201\256\350\252\277\346\225\264\345\200\244", Q_NULLPTR));
        li_base->setText(QString());
        li_waypoint_corr->setText(QString());
        li_all_corr->setText(QString());
        lb_all_corr->setText(QApplication::translate("Dialog_driving_adjustment", "\347\265\214\350\267\257\343\201\256\350\252\277\346\225\264\345\200\244\357\274\213\347\217\276\345\234\250\343\201\256\350\252\277\346\225\264\345\200\244", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Dialog_driving_adjustment: public Ui_Dialog_driving_adjustment {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_DRIVING_ADJUSTMENT_H
