/********************************************************************************
** Form generated from reading UI file 'dialog_popup_signal.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_POPUP_SIGNAL_H
#define UI_DIALOG_POPUP_SIGNAL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Dialog_popup_signal
{
public:
    QPushButton *bt_cancel;
    QPushButton *bt_end;
    QLabel *lb_text;

    void setupUi(QDialog *Dialog_popup_signal)
    {
        if (Dialog_popup_signal->objectName().isEmpty())
            Dialog_popup_signal->setObjectName(QStringLiteral("Dialog_popup_signal"));
        Dialog_popup_signal->resize(507, 309);
        bt_cancel = new QPushButton(Dialog_popup_signal);
        bt_cancel->setObjectName(QStringLiteral("bt_cancel"));
        bt_cancel->setGeometry(QRect(270, 230, 241, 81));
        QFont font;
        font.setPointSize(30);
        bt_cancel->setFont(font);
        bt_end = new QPushButton(Dialog_popup_signal);
        bt_end->setObjectName(QStringLiteral("bt_end"));
        bt_end->setGeometry(QRect(0, 230, 241, 81));
        bt_end->setFont(font);
        lb_text = new QLabel(Dialog_popup_signal);
        lb_text->setObjectName(QStringLiteral("lb_text"));
        lb_text->setGeometry(QRect(10, 10, 491, 201));
        QFont font1;
        font1.setPointSize(60);
        lb_text->setFont(font1);
        lb_text->setAlignment(Qt::AlignCenter);

        retranslateUi(Dialog_popup_signal);

        QMetaObject::connectSlotsByName(Dialog_popup_signal);
    } // setupUi

    void retranslateUi(QDialog *Dialog_popup_signal)
    {
        Dialog_popup_signal->setWindowTitle(QApplication::translate("Dialog_popup_signal", "Dialog", Q_NULLPTR));
        bt_cancel->setText(QApplication::translate("Dialog_popup_signal", "\343\202\255\343\203\243\343\203\263\343\202\273\343\203\253", Q_NULLPTR));
        bt_end->setText(QApplication::translate("Dialog_popup_signal", "OK", Q_NULLPTR));
        lb_text->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class Dialog_popup_signal: public Ui_Dialog_popup_signal {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_POPUP_SIGNAL_H
