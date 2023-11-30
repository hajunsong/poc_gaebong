QT += core gui widgets

CONFIG += c++11

SOURCES += \
	main.cpp \
	mainwindow.cpp \
	gripper/zimmergripper.cpp \
	customsettings.cpp \
	tcpsocket.cpp \

HEADERS += \
	mainwindow.h \
	gripper/zimmergripper.h \
	robot/libcustom/Signal.hpp \
	robot/libcustom/tcpclient.h \
	robot/libcustom/timer.h \
	robot/robotapi/robot.h \
#    robot/robotapi/ur10_v2.h \
#    robot/robotapi/m1013_v2.h \
	robot/robotapi/rb10_v2.h \
	robot/robotconf.h \
	robot/sdk.h \
	customsettings.h \
	tcpsocket.h \

FORMS += mainwindow.ui

LIBS += -L$$PWD/robot/ -lrobotsdkv2
LIBS += -lmodbus

INCLUDEPATH += $$PWD/robot
DEPENDPATH += $$PWD/robot
