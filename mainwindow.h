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
enum {Ready=0, Pick, Chuck1, Door1, LatheStart1, LatheWait1, UnChuck1, Place, RePick, Chuck2, Door2, LatheStart2, LatheWait2, UnChuck2, RePlace};
enum {TestPick=1, TestPlace, TestFinish};
const double duration_super_slow = 5;
const double duration_slow = 50;
const double duration_fast = 100;
const double ws_duration_slow = 20;

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
	void moveLathe();

    pthread_t move_wait_thread;
    char cmd_type;
	double cmd_value[6];
    bool move_complete, moving, cmd_flag;

    QTimer *mainTimer;
    unsigned char mainState, pocState, pocSubState;
    bool robot_init, gripper_init;
    int door_cnt, obj_cnt;

    int DO;
	typedef struct{
		unsigned char chuck, door, work;
		enum DOOR{DoorClose, DoorOpen};
		enum CHUCK{ChuckOpen, ChuckClose};
		enum WORK{WorkComplete, Working};
	}MachineState;

	MachineState mchState;
	unsigned char visionData[3];

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
	double JS_ready2pick3[6] = {2.56529, -1.66986, 2.21928, -2.12812, -1.57164, -1.4408};
	double WS_to_obj_z = (0.262829 - 0.37859);
    double JS_withdraw_pick[6] = {2.04024,-2.87771,2.05892,0.787739,1.56417,-3.17643};
	double Pick_obj_offset_x = 0.111;
	double Pick_obj_offset_y = 0.100;

	// positions related to inserting obj
	double JS_to_chuck1[6] = {1.51169, -2.66293, 2.10997, -1.57099, -1.57101, 2.88785e-05};
	double JS_to_chuck2[6] = {1.48943, -0.589587, 1.21643, -3.75866, -1.50547, 1.60286};
	double JS_to_chuck3[6] = {1.38757, -0.601745, 0.783276, -0.331743, -0.167804, -1.37842};
	double WS_to_chuck3[16] = {-0.00805969,0.0431334,0.999037,0.10123,
							   0.0419511,0.998204,-0.042759,-1.09495,
							   -0.999087,0.0415661,-0.00985471,0.307261,
							   0,0,0,1};
	double WS_to_chuck2[16] = {0.041437,0.998986,0.0175986,0.101181,
							   0.0105035,0.0171772,-0.999797,-1.09495,
							   -0.999086,0.0416135,-0.00978108,0.307264,
							   0,0,0,1};
	double WS_insert_x = (0.151233 - 0.10123) + 0.003;

    // position related to removing obj
	double JS_to_unchuck1[6] = {1.47679, -2.22999, 2.11595, 0.66896, 1.52501, -1.62673};
	double JS_to_unchuck3[6] = {1.68786, -0.909078, 1.09137, 0.374452, 1.70225, -1.51897};
	double WS_remove_x = (0.363416 - 0.27414);

    // positions related to placing obj
	double JS_over_place[6] = {3.65685, -1.62448, 2.46088, -2.41849, -1.58067, -1.05112};
	double WS_place_z = (0.23135 - 0.255064);

    // positions related to re-picking obj
	double JS_ready2repick1[6] = {2.91941, -2.05606, 2.20549, -0.247987, -0.210816, 3.23333};
	double JS_ready2repick3[6] = {2.69954, -1.61074, 2.6411, -1.06704, -0.444182, 3.16927};
	double WS_repick_z = (0.0507944 - 0.183111);
	double WS_to_repick[16] = {0.99994,0.00874661,0.00661316,0.324467,
							   -0.00654473,-0.00782962,0.999948,0.156472,
							   0.00879793,-0.999931,-0.0077719,0.0507944,
							   0,0,0,1};

    // positions related to re-chuck obj
	double JS_to_rechuck1[6] = {1.51169, -2.66293, 2.10997, -1.57099, -1.57101, 2.88785e-05};
	double JS_to_rechuck3[6] = {1.40569, -0.589763, 0.761794, -0.345699, -0.18133, 1.68357};
	double WS_reinsert_x = (0.152136 - 0.10199) + 0.003;
	double WS_to_rechuck3[16] = {-0.00906411,0.0181906,0.999793,0.10199,
								 0.0417493,0.99897,-0.0177971,-1.09766,
								 -0.999087,0.0415794,-0.00981422,0.306237,
								 0,0,0,1};

    // positions related to re-remove obj
	double JS_to_reunchuck1[6] = {1.51169, -2.66293, 2.10997, -1.57099, -1.57101, 2.88785e-05};
	double JS_to_reunchuck3[6] = {1.39644, -0.587475, 0.75675, -0.334689, -0.190377, 1.67535};

	double WS_to_reunchuck3[16] = {-0.00906411,0.0181906,0.999793,0.10199,
								 0.0417493,0.99897,-0.0177971,-1.09766,
								 -0.999087,0.0415794,-0.00981422,0.306237,
								 0,0,0,1};
	double WS_to_reunchuck4[16] = {-0.00907104,0.0181994,0.999793,0.102338,
								   0.0416771,0.998972,-0.0178063,-1.098,
								   -0.99909,0.041507,-0.00982022,0.306135,
								   0,0,0,1};
	double WS_reremove_x = (0.15171 - 0.10199);

    // positions related to re-placing obj
	double JS_over_replace[6] = {3.18828, -1.65712, 2.17651, -2.10261, -1.57818, -0.81763};
	double WS_replace_z = (0.261982 - 0.394659);
	double Place_obj_offset_x = 0.111;
	double Place_obj_offset_y = 0.100;

	// positions related to start
	double JS_to_start_SW[6] = {-0.180043, -2.68893, 1.82169, -0.70945, -1.5229, -1.69509};
	double JS_over_MEM_SW[6] = {1.8513, -1.67913, 2.09128, -1.98714, -1.55504, -1.29788};
	double WS_push_MEM_SW_x = (0.391132 - 0.330573);
	double JS_over_start_SW[6] = {0.609222, -1.38275, 2.16151, -2.35237, -1.61244, -0.953523};
	double WS_push_start_SW_y = (-0.516621 - -0.507134);

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
