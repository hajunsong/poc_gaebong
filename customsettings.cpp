#include "ui_mainwindow.h"
#include "customsettings.h"

CustomSettings::CustomSettings(void *_ui)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
}

void CustomSettings::saveConfigFile()
{
    QSettings settings(configFile(), QSettings::IniFormat);

    QString robot_ip = static_cast<Ui::MainWindow*>(ui)->txtRobotIP->text();
    QString robot_port = static_cast<Ui::MainWindow*>(ui)->txtRobotPORT->text();
    QString gripper_ip = static_cast<Ui::MainWindow*>(ui)->txtGripperIP->text();
    QString gripper_port = static_cast<Ui::MainWindow*>(ui)->txtGripperPORT->text();
//    int window_width = static_cast<Ui::MainWindow*>(ui)->centralwidget->geometry().width();
//    int window_height = static_cast<Ui::MainWindow*>(ui)->centralwidget->geometry().height();
	QString vision_port = static_cast<Ui::MainWindow*>(ui)->txtVisionPORT->text();

    settings.setValue("ROBOT_IP", robot_ip);
    settings.setValue("ROBOT_PORT", robot_port);
    settings.setValue("GRIPPER_IP", gripper_ip);
    settings.setValue("GRIPPER_PORT", gripper_port);
	settings.setValue("VISION_PORT", vision_port);
//    settings.setValue("WINDOW_W", window_width);
//    settings.setValue("WINDOW_H", window_height);

    settings.sync();
}

void CustomSettings::loadConfigFile()
{
    if (!QFile::exists(configFile())) return;

    QSettings settings(configFile(), QSettings::IniFormat);

    if(!settings.value("ROBOT_IP").isNull()){
        QString ip = settings.value("ROBOT_IP").toString();
        static_cast<Ui::MainWindow*>(ui)->txtRobotIP->setText(ip);
    }
    if(!settings.value("ROBOT_PORT").isNull()){
        QString port = settings.value("ROBOT_PORT").toString();
        static_cast<Ui::MainWindow*>(ui)->txtRobotPORT->setText(port);
    }
    if(!settings.value("GRIPPER_IP").isNull()){
        QString gripper_ip = settings.value("GRIPPER_IP").toString();
        static_cast<Ui::MainWindow*>(ui)->txtGripperIP->setText(gripper_ip);
    }
    if(!settings.value("GRIPPER_PORT").isNull()){
        QString gripper_port = settings.value("GRIPPER_PORT").toString();
        static_cast<Ui::MainWindow*>(ui)->txtGripperPORT->setText(gripper_port);
    }
	if(!settings.value("VISION_PORT").isNull()){
		QString vision_port = settings.value("VISION_PORT").toString();
		static_cast<Ui::MainWindow*>(ui)->txtVisionPORT->setText(vision_port);
	}
//    int w = 0, h = 0;
//    if(!settings.value("WINDOW_W").isNull()){
//        w = settings.value("WINDOW_W").toInt();
//    }
//    if(!settings.value("WINDOW_H").isNull()){
//        h = settings.value("WINDOW_H").toInt();
//    }
//    if(w > 0 && h > 0){
//        static_cast<Ui::MainWindow*>(ui)->centralwidget->setFixedSize(w, h);
//    }
}

QString CustomSettings::configFile()
{
    QString filePath = qApp->applicationDirPath() + "/config.ini";
    return filePath;
}

