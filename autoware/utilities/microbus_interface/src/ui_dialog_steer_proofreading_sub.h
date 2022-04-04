/********************************************************************************
** Form generated from reading UI file 'dialog_steer_proofreading_sub.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_STEER_PROOFREADING_SUB_H
#define UI_DIALOG_STEER_PROOFREADING_SUB_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_dialog_steer_proofreading_sub
{
public:
    QLabel *label;
    QLabel *lb_text;
    QPushButton *bt_ok;
    QPushButton *bt_cancel;
    QLineEdit *tx_straight_sub_voltage;

    void setupUi(QDialog *dialog_steer_proofreading_sub)
    {
        if (dialog_steer_proofreading_sub->objectName().isEmpty())
            dialog_steer_proofreading_sub->setObjectName(QStringLiteral("dialog_steer_proofreading_sub"));
        dialog_steer_proofreading_sub->resize(565, 619);
        label = new QLabel(dialog_steer_proofreading_sub);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(0, 410, 561, 51));
        QFont font;
        font.setPointSize(30);
        label->setFont(font);
        label->setLayoutDirection(Qt::LeftToRight);
        label->setAlignment(Qt::AlignCenter);
        lb_text = new QLabel(dialog_steer_proofreading_sub);
        lb_text->setObjectName(QStringLiteral("lb_text"));
        lb_text->setGeometry(QRect(0, 10, 561, 391));
        lb_text->setFont(font);
        lb_text->setLayoutDirection(Qt::LeftToRight);
        lb_text->setAlignment(Qt::AlignCenter);
        bt_ok = new QPushButton(dialog_steer_proofreading_sub);
        bt_ok->setObjectName(QStringLiteral("bt_ok"));
        bt_ok->setGeometry(QRect(60, 550, 201, 61));
        bt_ok->setFont(font);
        bt_cancel = new QPushButton(dialog_steer_proofreading_sub);
        bt_cancel->setObjectName(QStringLiteral("bt_cancel"));
        bt_cancel->setGeometry(QRect(300, 550, 201, 61));
        bt_cancel->setFont(font);
        tx_straight_sub_voltage = new QLineEdit(dialog_steer_proofreading_sub);
        tx_straight_sub_voltage->setObjectName(QStringLiteral("tx_straight_sub_voltage"));
        tx_straight_sub_voltage->setGeometry(QRect(162, 464, 221, 81));
        tx_straight_sub_voltage->setFont(font);

        retranslateUi(dialog_steer_proofreading_sub);

        QMetaObject::connectSlotsByName(dialog_steer_proofreading_sub);
    } // setupUi

    void retranslateUi(QDialog *dialog_steer_proofreading_sub)
    {
        dialog_steer_proofreading_sub->setWindowTitle(QApplication::translate("dialog_steer_proofreading_sub", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("dialog_steer_proofreading_sub", "\347\233\264\351\200\262\350\265\260\350\241\214\346\231\202\343\201\256\343\202\265\343\203\226\351\233\273\345\234\247\345\200\244\343\202\222\345\205\245\345\212\233", Q_NULLPTR));
        lb_text->setText(QString());
        bt_ok->setText(QApplication::translate("dialog_steer_proofreading_sub", "OK", Q_NULLPTR));
        bt_cancel->setText(QApplication::translate("dialog_steer_proofreading_sub", "\343\202\255\343\203\243\343\203\263\343\202\273\343\203\253", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class dialog_steer_proofreading_sub: public Ui_dialog_steer_proofreading_sub {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_STEER_PROOFREADING_SUB_H
