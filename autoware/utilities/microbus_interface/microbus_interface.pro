#-------------------------------------------------
#
# Project created by QtCreator 2019-09-10T11:06:21
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = microbus_interface
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
    src/main.cpp \
    src/mainwindow.cpp \
    src/dialog_driving_adjustment.cpp \
    src/dialog_popup_signal.cpp \
    src/dialog_steer_proofreading_sub.cpp \
    src/dialog_rosbag.cpp

HEADERS += \
    src/mainwindow.h \
    src/dialog_driving_adjustment.h \
    src/dialog_popup_signal.h \
    src/dialog_steer_proofreading_sub.h \
    src/dialog_steer_proofreading_sub.h \
    src/dialog_rosbag.h

FORMS += \
    src/mainwindow.ui \
    src/dialog_driving_adjustment.ui \
    src/dialog_popup_signal.ui \
    src/dialog_steer_proofreading_sub.ui \
    src/dialog_rosbag.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
