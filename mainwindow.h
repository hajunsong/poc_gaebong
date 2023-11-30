#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLineEdit>
#include <QtDebug>

#include <pthread.h>

#include "sdk.h"
#include "gripper/zimmergripper.h"
#include "customsettings.h"
#include "tcpsocket.h"

/*
 * Zimmer Gripper : 192.168.0.253, 502
 * This PC        : 192.168.0.100
 * UR 10e         : 192.168.0.7, 30003
*/

enum {Wait=0, Init, Start, TestStart};
enum {None=0, MoveJ, MoveL, GripOn, GripOff, GripCustom, ChuckOpen, ChuckClose, DoorOpen, DoorClose, LathWait};
enum {Ready=0, Pick, Chuck1, Door1, LathStart1, LatheWait1, UnChuck1, Place, RePick, Chuck2, Door2, LatheStart2, LatheWait2, UnChuck2, RePlace};
enum {TestPick=1, TestPlace, TestFinish};
const double duration_super_slow = 5;
const double duration_slow = 30;
const double duration_fast = 100;

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    static void *move_wait_func(void *arg);

private:
    Ui::MainWindow *ui;
    CustomSettings *customSettings;

    sdk_info robotInfor;
    RConf Conf;
    ZimmerGripper *gripper;

	TCP::TcpSocket *tcpSocket;

	bool robot_connected, gripper_connected, vision_connected;

    double mat[9], pos[3], jnt[6], state;
    std::array<QLineEdit*, 6> txtJoint, txtPose;

    uint16_t gripper_write_reg[NUM_SEND_REG];
    uint16_t gripper_read_reg[NUM_RECV_REG];

    void moveJoint(double cmd[6], double vel = 50);
    void movePose(double cmd[6], double vel = 50, std::string opt = "abs", std::string coord = "base");
    void moveGripperOn();
    void moveGripperOff();
    void moveGripperCustom();
    void moveChuckOpen();
    void moveChuckClose();
    void moveDoorOpen();
    void moveDoorClose();
	void moveLath();

    pthread_t move_wait_thread;
    char cmd_type;
	double cmd_value[6];
    bool move_complete, moving, cmd_flag;

    QTimer *mainTimer;
    unsigned char mainState, pocState, pocSubState;
    bool robot_init, gripper_init;
    int door_cnt, obj_cnt;

    int DI;
    int DO;
    bool door_close;
    bool chuck_open, chuck_close, chuck_moving;

    void POCControlFunc();
    bool POCReady();
    bool POCPickObj();
	bool POCChuck1Obj();
	bool POCDoorSwitch1();
	bool POCLatheStart1();
	bool POCLatheWait1();
    bool POCUnChuck1Obj();
    bool POCPlaceObj();
    bool POCRePickObj();
    bool POCChuck2Obj();
    bool POCUnChuck2Obj();
	bool POCRePlaceObj();

    void POCTestFunc();
    bool POCTestPick();
    bool POCTestPlace();

public slots:
    // button event
    void btnRobotConnectClicked();
    void btnGripperConnectClicked();
    void btnInitClicked();
    void btnRunClicked();
    void btnPickObjClicked();
    void btnChuckObjClicked();
    void btnUnChuckObjClicked();
    void btnDoorSwitchClicked();
    void btnPlaceObjClicked();
    void btnRePickObjClicked();
    void btnChuckObj2Clicked();
    void btnUnChuckObj2Clicked();
    void btnDoorSwitch2Clicked();
    void btnRePlaceObjClicked();
    void btnPrintClicked();
    void btnGripperInitClicked();
    void btnGripperGripClicked();
    void btnGripperReleaseClicked();
    void btnTestClicked();
	void btnVisionListenClicked();
	void btnLathStartClicked();
	void btnLathWaitClicked();

    // timer timeout event
	void update();

	// custom event
    void robotStateUpdate();
    void gripperStateUpdate();
	void visionStateUpdate();

signals:
    void robotUpdate();
    void gripperUpdate();
	void visionUpdate();

private:
    // define positions
    double JS_ready[6] = {M_PI_4, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, 0};
//    double JS_ready[6] = {M_PI, 0, 0, 0, 0, 0};

    // positions related to door
    double JS_to_door_SW2[6] = {1.4337400197982788,-2.6825778484344482,1.8206619024276733,-0.7187622785568237,-1.5197819471359253,-1.6940726041793823};
    double JS_over_door_SW[6] = {1.8122, -1.24393, 1.54319, -1.87347, -1.55605, -1.33591};
    double WS_push_SW_x = (0.390025 - 0.378888);

    // positions related to picking obj
	double JS_ready2pick2[6] = {3.14159, -2.66281, 2.10999, -1.57083, -1.57082, 0};
	double JS_ready2pick3[6] = {2.60827, -1.68614, 2.22551, -2.11943, -1.57174, -1.39789};
	double WS_to_obj_z = (0.256354 - 0.382611);
    double JS_withdraw_pick[6] = {2.04024,-2.87771,2.05892,0.787739,1.56417,-3.17643};
    double Pick_obj_offset_x = 0.134;
    double Pick_obj_offset_y = 0.155;

    // positions related to inserting obj
    double JS_to_chuck1[6] = {2.88659, -2.66293, 2.10993, -1.57099, -1.57105, 0};
    double JS_to_chuck3[6] = {2.85466, -0.885332, 1.29192, -0.474272, -0.291698, -4.64355};
	double WS_insert_y = (0.0885091 - 0.0245522) - 0.0015;

    // position related to removing obj
    double JS_to_unchuck1[6] = {3.05115, -2.23001, 2.11564, 0.668588, 1.52513, -1.60452};
    double JS_to_unchuck3[6] = {3.21681, -1.24966, 1.57683, 0.230587, 1.6592, -1.54682};
    double WS_remove_y = (0.297258 - 0.223869);

    // positions related to placing obj
    double JS_over_place[6] = {0.165977, -1.23116, 1.69105, 1.09511, 1.57986, -2.96114};
    double WS_place_z = (0.267641 - 0.388906);

    // positions related to re-picking obj
    double JS_ready2repick1[6] = {0.229675, -2.10409, 2.13163, 0.610423, 1.70094, -3.01387};
    double JS_ready2repick3[6] = {0.384779, -1.58145, 2.58984, -1.00666, 1.95014, -3.14863};
    double WS_repick_z = (0.0595135 - 0.190523);

    // positions related to re-chuck obj
    double JS_to_rechuck1[6] = {2.88659, -2.66293, 2.10993, -1.57099, -1.57105, -3.26079};
    double JS_to_rechuck3[6] = {2.82928, -0.888926, 1.29953, -0.474195, -0.302735, -4.64735};
	double WS_reinsert_y = (0.087292 - 0.0013317);

    // positions related to re-remove obj
    double JS_to_reunchuck1[6] = {2.88659, -2.66293, 2.10993, -1.57099, -1.57105, -3.26079};
    double JS_to_reunchuck3[6] = {2.8402, -0.880683, 1.28807, -0.47079, -0.306735, -4.64744};
    double WS_reremove_y = (0.0921894 - 0.0100897);

    // positions related to re-placing obj
    double JS_over_replace[6] = {0.717507, -1.64829, 2.25819, -2.18686, -1.56986, -1.61307};
    double WS_replace_z = (0.24291 - 0.350613);
    double Place_obj_offset_x = 0.134;
    double Place_obj_offset_y = 0.155;

	// positions related to start
	double JS_over_MEM_SW[6] = {1.8513, -1.67913, 2.09128, -1.98714, -1.55504, -1.29788};
	double WS_push_MEM_SW_x = (0.391132 - 0.330573);
	double JS_over_start_SW[6] = {1.90774, -1.56634, 2.09153, -2.1018, -1.55486, -1.24263,};
	double WS_push_start_SW_x = (0.382109 - 0.371739);

	// positions related to wait lath
	double JS_lathwait[6] = {1.8122, -1.24393, 1.54319, -1.87347, -1.55605, -1.33591};
	double WS_lathwait[16] = {0,0,-1,0.00520054,
							  -1,0,0,-0.617677,
							  0,-1,0,0.341776,
							  0,0,0,1};

//	0.0692782, -1.13185, 2.30652, -2.32341, 0.0637945, -2.01098,
//	-0.573819, -0.330021, 0.16719,
//	-0.999249,-0.0148681,0.0357897,
//	-0.0347484,-0.0652133,-0.997266,
//	0.0171614,-0.997761,0.0646476,


	double JS_over_JOG_SW[6] = {1.92447, -1.61531, 2.07528, -2.03703, -1.55524, -1.22519};
	double WS_push_JOG_SW_x = {0.390333 - 0.376223};

    // test position related to picking obj
//    double TJS_ready2pick2[6] = {0.297308,-2.71073317527771,2.1584970951080322,-1.0338462591171265,1.5014338493347168,-3.1651346683502197};
    double TJS_ready2pick2[6] = {0, M_PI_2, 0, 0, 0, 0};
    double TJS_ready2pick3[6] = {0.577095, -1.23603, 2.29238, -1.06028, 0.574473, -3.14432};
    double TWS_to_obj_z = (0.0410696 - 0.144147);
    double TJS_withdraw_pick[6] = {1.84024,-2.87771,2.05892,0.787739,1.56417,-3.17643};
    double TPick_obj_offset_x = 0.134;
    double TPick_obj_offset_y = 0.155;

    // test position related to placing obj
    double TJS_over_place[6] = {1.76592, -0.727673, 1.32078, -0.596243, 1.76337, -3.14399};
    double TWS_place_z = (0.0469771 - 0.150919);
    double TPlace_obj_offset_x = 0.134;
    double TPlace_obj_offset_y = 0.155;
};

#endif // MAINWINDOW_H
