#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	system("pause");

    gripper = new ZimmerGripper();
    memset(gripper_write_reg, 0, sizeof(uint16_t)*NUM_SEND_REG);
    memset(gripper_read_reg, 0, sizeof(uint16_t)*NUM_RECV_REG);

    connect(ui->btnRobotConnect, SIGNAL(clicked()), this, SLOT(btnRobotConnectClicked()));
    connect(ui->btnGripperConnect, SIGNAL(clicked()), this, SLOT(btnGripperConnectClicked()));
    connect(ui->btnInit, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));

    connect(ui->btnPickObj, SIGNAL(clicked()), this, SLOT(btnPickObjClicked()));
    connect(ui->btnChuckObj, SIGNAL(clicked()), this, SLOT(btnChuckObjClicked()));
    connect(ui->btnUnChuckObj, SIGNAL(clicked()), this, SLOT(btnUnChuckObjClicked()));
    connect(ui->btnDoorSwitch, SIGNAL(clicked()), this, SLOT(btnDoorSwitchClicked()));
    connect(ui->btnPlaceObj, SIGNAL(clicked()), this, SLOT(btnPlaceObjClicked()));
    connect(ui->btnRePickObj, SIGNAL(clicked()), this, SLOT(btnRePickObjClicked()));
    connect(ui->btnChuckObj2, SIGNAL(clicked()), this, SLOT(btnChuckObj2Clicked()));
    connect(ui->btnUnChuckObj2, SIGNAL(clicked()), this, SLOT(btnUnChuckObj2Clicked()));
    connect(ui->btnDoorSwitch2, SIGNAL(clicked()), this, SLOT(btnDoorSwitch2Clicked()));
    connect(ui->btnRePlaceObj, SIGNAL(clicked()), this, SLOT(btnRePlaceObjClicked()));
	connect(ui->btnLathStart, SIGNAL(clicked()), this, SLOT(btnLathStartClicked()));
	connect(ui->btnLathWait, SIGNAL(clicked()), this, SLOT(btnLathWaitClicked()));

    connect(ui->btnPrint, SIGNAL(clicked()), this, SLOT(btnPrintClicked()));

    connect(ui->btnGripperInit, SIGNAL(clicked()), this, SLOT(btnGripperInitClicked()));
    connect(ui->btnGripperGrip, SIGNAL(clicked()), this, SLOT(btnGripperGripClicked()));
	connect(ui->btnGripperRelease, SIGNAL(clicked()), this, SLOT(btnGripperReleaseClicked()));

    connect(this, SIGNAL(robotUpdate()), this, SLOT(robotStateUpdate()));
    connect(this, SIGNAL(gripperUpdate()), this, SLOT(gripperStateUpdate()));

    connect(ui->btnTest, SIGNAL(clicked()), this, SLOT(btnTestClicked()));

    txtJoint = {ui->txtJoint1, ui->txtJoint2, ui->txtJoint3, ui->txtJoint4, ui->txtJoint5, ui->txtJoint6};
    txtPose = {ui->txtPose1, ui->txtPose2, ui->txtPose3, ui->txtPose4, ui->txtPose5, ui->txtPose6};

    mainState = -1;
    pocState = -1;
    pocSubState = -1;

    robot_init = false;
    gripper_init = false;
    robot_connected = false;
    gripper_connected = false;

    move_complete = false;
    moving = false;
    cmd_flag = false;

    DO = 0;

    door_cnt = 0;
    obj_cnt = 0;

	memset(&mchState, 0, 3);

    mainTimer = new QTimer(this);
    mainTimer->setInterval(10);
    connect(mainTimer, SIGNAL(timeout()), this, SLOT(update()));
    mainTimer->start();

	tcpSocket = new TCP::TcpSocket();
	connect(this, SIGNAL(visionUpdate()), this, SLOT(visionStateUpdate()));
	connect(ui->btnVisionListen, SIGNAL(clicked()), this, SLOT(btnVisionListenClicked()));
	memset(visionData, 0, 3);

	customSettings = new CustomSettings(ui);
	customSettings->loadConfigFile();
}

MainWindow::~MainWindow()
{
	customSettings->saveConfigFile();
	delete tcpSocket;
    delete customSettings;
    delete ui;
}

void MainWindow::update()
{
	if(robot_connected){
		emit robotUpdate();
	}

	if(gripper_connected){
		emit gripperUpdate();
	}

	if(tcpSocket->isConnected()){
		emit visionUpdate();
	}

    if(robot_connected && gripper_connected){
        switch(mainState){
            case Wait:
            {
                if(pocState > 0) {
				   mainState = Start;
                   // mainState = TestStart;
                }
                break;
            }
            case Init:
            {
				if(!robot_init){
//                    moveJoint(JS_ready, duration_slow);
					robot_init = true;
				}
				if(!gripper_init && robot_init){
					gripper->gripper_init();
					gripper_init = true;
				}
				pocState = 0;
                pocSubState = 0;
                ui->rbInit->setChecked(true);
				obj_cnt = 0;
                break;
            }
            case Start:
            {
                POCControlFunc();
//                std::cout << (int)obj_cnt << std::endl;
                break;
            }
            case TestStart:
            {
                POCTestFunc();
                break;
            }
            default:
                break;
        }
    }
}

void MainWindow::POCControlFunc()
{
	std::cout << "pocState : " << (int)pocState << ", ";
	std::cout << "pocSubState : " << (int)pocSubState << std::endl;
    switch(pocState){
        case Ready:
        {
            if(POCReady()){
                pocState = Ready;
                pocSubState = 0;
            }
            break;
        }
        case Pick:
        {
            if(POCPickObj()){
                pocState = Chuck1;
                pocSubState = 0;
            }
            ui->rbPick->setChecked(true);
            break;
        }
        case Chuck1:
        {
            if(POCChuck1Obj()){
                pocState = Door1;
                pocSubState = 0;
            }
            ui->rbChuck1->setChecked(true);
            break;
        }
        case Door1:
        {
            if(POCDoorSwitch1()){
                if(door_cnt%2 == 0) {
                    pocState = UnChuck1;
                }
                else{
					pocState = LatheStart1;
                }
                pocSubState = 0;
            }
            ui->rbDoor1->setChecked(true);
            break;
        }
		case LatheStart1:
		{
			if(POCLatheStart1()){
				pocState = LatheWait1;
				pocSubState = 0;
			}
			ui->rbLathStart->setChecked(true);
			break;
		}
        case LatheWait1:
        {
			if(POCLatheWait1()){
				pocState = Door1;
				pocSubState = 0;
			}
			ui->rbLathWait->setChecked(true);
            break;
        }
        case UnChuck1:
        {
            if(POCUnChuck1Obj()){
                pocState = Place;
                pocSubState = 0;
            }
            ui->rbChuck1->setChecked(true);
            break;
        }
        case Place:
        {
            if(POCPlaceObj()){
                pocState = RePick;
                pocSubState = 0;
            }
            ui->rbPlace->setChecked(true);
            break;
        }
        case RePick:
        {
            if(POCRePickObj()){
                pocState = Chuck2;
                pocSubState = 0;
            }
            ui->rbRePick->setChecked(true);
            break;
        }
        case Chuck2:
        {
            if(POCChuck2Obj()){
                pocState = Door2;
                pocSubState = 0;
            }
            ui->rbChuck2->setChecked(true);
            break;
        }
        case Door2:
        {
            if(POCDoorSwitch1()){
                if(door_cnt%2 == 0) {
                    pocState = UnChuck2;
                }
                else{
					pocState = LatheStart2;
                }
                pocSubState = 0;
            }
            ui->rbDoor2->setChecked(true);
            break;
        }
		case LatheStart2:
		{
			if(POCLatheStart1()){
				pocState = LatheWait2;
				pocSubState = 0;
			}
			ui->rbLathStart->setChecked(true);
			break;
		}
        case LatheWait2:
        {
			if(POCLatheWait1()){
				pocState = Door2;
				pocSubState = 0;
			}
			ui->rbLathWait->setChecked(true);
            break;
        }
        case UnChuck2:
        {
            if(POCUnChuck2Obj()){
                pocState = RePlace;
                pocSubState = 0;
            }
            ui->rbChuck2->setChecked(true);
            break;
        }
        case RePlace:
        {
            if(POCRePlaceObj()){
                obj_cnt++;
                if(obj_cnt == 1) obj_cnt = 2;
                else if(obj_cnt == 3) obj_cnt = 9;
                else if(obj_cnt == 10) obj_cnt = 11;

                pocState = obj_cnt < 12 ? Pick : Ready;
                pocSubState = 0;
            }
            ui->rbRePlace->setChecked(true);
            break;
        }
        default:
            break;
    }
}

bool MainWindow::POCReady(){
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
				moveJoint(JS_ready2pick2, duration_fast);
                break;
            default:
                finish = true;
                break;
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCPickObj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_ready2pick2, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(JS_ready2pick3, duration_slow);
                break;
            }
            case 2:
            {
                moveGripperOff();
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, 0, 0, 0, 0};
				offset[0] = Pick_obj_offset_x*(obj_cnt/3);
				offset[1] = Pick_obj_offset_y*(obj_cnt%3);
                movePose(offset, duration_fast, "rel");
                break;
            }
            case 4:
            {
                double offset[6] = {0, 0, WS_to_obj_z, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 5:
            {
                moveGripperOn();
                break;
            }
            case 6:
            {
                double offset[6] = {0, 0, -WS_to_obj_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            case 7:
            {
                moveJoint(JS_ready2pick2, duration_fast);
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCChuck1Obj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_to_chuck1, duration_fast);
                break;
            }
            case 1:
			{
				moveJoint(JS_to_chuck2, duration_fast);
                break;
            }
            case 2:
            {
//                moveJoint(JS_to_chuck3, duration_slow);
				movePose(WS_to_chuck3, ws_duration_slow);
                break;
            }
            case 3:
            {
				double offset[6] = {WS_insert_x, 0, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");

                break;
            }
            case 4: // foot switch
            {
				moveChuckClose();
                break;
            }
            case 5:
            {
				moveGripperOff();
                break;
            }
            case 6:
            {
				double offset[6] = {-WS_insert_x, 0, 0, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
			}
			case 7:
			{
				movePose(WS_to_chuck2, ws_duration_slow);
				break;
			}
			case 8:
			{
				moveJoint(JS_to_chuck1, duration_slow);
				break;
			}
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCDoorSwitch1()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
				if(door_cnt%2 == 0){
					moveDoorClose();
				}
				if(door_cnt%2 == 1){
					moveDoorOpen();
				}
                break;
			}
            default:
            {
                door_cnt++;
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

	return finish;
}

bool MainWindow::POCLatheStart1()
{
	bool finish = false;

	if(!cmd_flag){
		switch(pocSubState){
			case 0:
			{
				moveJoint(JS_to_start_SW, duration_fast);
				break;
			}
			case 1:
			{
//				double offset[6] = {WS_push_MEM_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 2:
			{
//				double offset[6] = {-WS_push_MEM_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 3:
			{
				moveJoint(JS_over_start_SW, duration_fast);
				break;
			}
			case 4:
			{
				double offset[6] = {0, WS_push_start_SW_y, 0, 0, 0, 0};
				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 5:
			{
				double offset[6] = {0, -WS_push_start_SW_y, 0, 0, 0, 0};
				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 6:
			{
				moveJoint(JS_to_start_SW, duration_fast);
				break;
			}
			case 7:
			{
				moveJoint(JS_to_chuck1, duration_fast);
				break;
			}
			default:
			{
				finish = true;
				break;
			}
		}
		pocSubState++;
	}

	return finish;
}

bool MainWindow::POCLatheWait1()
{
	bool finish = false;

	if(!cmd_flag){
		switch(pocSubState){
			case 0:
			{
//				movePose(WS_lathwait, duration_slow, "abs", "base");
				break;
			}
			case 1:
			{
//				tcpSocket->sendData('1');
				break;
			}
			case 2:
			{
				moveLathe();
				break;
			}
//			case 3:
//			{
//				moveJoint(JS_over_JOG_SW, duration_fast);
//				break;
//			}
//			case 4:
//			{
//				double offset[6] = {WS_push_JOG_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
//				break;
//			}
//			case 5:
//			{
//				double offset[6] = {-WS_push_JOG_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
//				break;
//			}
//			case 6:
//			{
//				moveJoint(JS_ready2pick3, duration_fast);
//				break;
//			}
			default:
			{
				finish = true;
				break;
			}
		}
		pocSubState++;
	}

	return finish;
}

bool MainWindow::POCUnChuck1Obj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_to_unchuck1, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(JS_to_unchuck3, duration_fast);
                break;
            }
            case 2:
            {
				double offset[6] = {WS_remove_x, 0, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 3:
            {
                moveGripperOn();
                break;
            }
            case 4: // foot switch
            {
                moveChuckOpen();
                break;
            }
            case 5:
            {
				double offset[6] = {-WS_remove_x, 0, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 6:
            {
                moveJoint(JS_to_unchuck1, duration_fast);
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCPlaceObj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
//                moveJoint(JS_withdraw_pick, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(JS_over_place, duration_fast);
                break;
            }
            case 2:
            {
                double offset[6] = {0, 0, WS_place_z, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 3:
            {
                moveGripperOff();
                break;
            }
            case 4:
            {
				double offset[6] = {0, 0, -WS_place_z + 0.03, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCRePickObj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
				moveJoint(JS_ready2repick1, duration_fast);
                break;
            }
            case 1:
            {
				moveJoint(JS_ready2repick3, duration_fast);
//				movePose(WS_to_repick, duration_super_slow);
                break;
            }
            case 2:
            {
                moveGripperOff();
                break;
            }
            case 3:
			{
				movePose(WS_to_repick, duration_super_slow);
                break;
            }
            case 4:
            {
                moveGripperOn();
                break;
            }
            case 5:
            {
				double offset[6] = {0, 0, 0.05, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            case 6:
            {
				moveJoint(JS_ready2pick2, duration_fast);
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCChuck2Obj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
				moveJoint(JS_to_chuck1, duration_fast);
                break;
            }
            case 1:
            {
				moveJoint(JS_to_chuck2, duration_slow);
                break;
            }
            case 2:
            {
//				moveJoint(JS_to_chuck3, duration_slow);
				movePose(WS_to_rechuck3, ws_duration_slow);
                break;
            }
            case 3:
            {
				double offset[6] = {WS_reinsert_x, 0, 0, 0, 0, 0};
				movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 4: // foot switch
            {
                moveChuckClose();
                break;
            }
            case 5:
            {
                moveGripperOff();
                break;
            }
            case 6:
            {
				double offset[6] = {-WS_reinsert_x, 0, 0, 0, 0, 0};
				movePose(offset, duration_fast , "rel");
                break;
            }
			case 7:
			{
				movePose(WS_to_chuck2, ws_duration_slow);
				break;
			}
			case 8:
            {
				moveJoint(JS_to_chuck1, duration_slow);
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCUnChuck2Obj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_to_reunchuck1, duration_fast);
                break;
            }
            case 1:
            {
				moveJoint(JS_to_chuck2, duration_fast);
                break;
            }
            case 2:
            {
				movePose(WS_to_reunchuck3, ws_duration_slow);
//                moveJoint(JS_to_reunchuck3, duration_slow);
                break;
            }
            case 3:
            {
				double offset[6] = {WS_reremove_x, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");

                break;
            }
            case 4:
            {
                moveGripperOn();
                break;
            }
            case 5: // foot switch
            {
                moveChuckOpen();
                break;
            }
            case 6:
            {
//				double offset[6] = {-WS_reremove_x, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");

				movePose(WS_to_reunchuck4, duration_super_slow);
                break;
			}
			case 7:
			{
				movePose(WS_to_chuck2, ws_duration_slow);
				break;
			}
			case 8:
            {
				moveJoint(JS_to_reunchuck1, duration_fast);
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCRePlaceObj(){
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
//                moveJoint(JS_ready2pick2, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(JS_over_replace, duration_fast);
                break;
            }
            case 2:
            {
				double offset[6] = {0, 0, 0, 0, 0, 0};
				offset[0] = Pick_obj_offset_x*(obj_cnt/3);
				offset[1] = Pick_obj_offset_y*(obj_cnt%3);
                movePose(offset, duration_fast, "rel");
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, WS_replace_z, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 4:
            {
                moveGripperOff();
                break;
            }
            case 5:
            {
                double offset[6] = {0, 0, -WS_replace_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

void MainWindow::moveJoint(double cmd[], double vel)
{
    if(!cmd_flag){
        SetVelocity(vel);
        usleep(10000);
        movej(cmd);
        cmd_type = MoveJ;
        memcpy(cmd_value, cmd, sizeof(double)*6);
        pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::movePose(double *cmd, double vel, std::string opt, std::string coord)
{
    if(!cmd_flag){
        SetVelocity(vel);
        usleep(10000);

        if(coord == "tcp"){
            if(opt == "rel"){
//                for(unsigned int i = 0; i < 3; i++){
//                    cmd[i] += pos[i];
//                }
                double cmd_mat[16] = {0,};
//                for(unsigned int i = 0; i < 3; i++){
//                    for(unsigned int j = 0; j < 3; j++){
//                        cmd_mat[i*4 + j] = mat[i*3 + j];
//                    }
//                }

                cmd_mat[0] = 1; cmd_mat[1] = 0; cmd_mat[2] = 0;
                cmd_mat[4] = 0; cmd_mat[5] = 1; cmd_mat[6] = 0;
                cmd_mat[8] = 0; cmd_mat[9] = 0; cmd_mat[10] = 1;

                cmd_mat[0*4 + 3] = cmd[0];
                cmd_mat[1*4 + 3] = cmd[1];
                cmd_mat[2*4 + 3] = cmd[2];
                cmd_mat[15] = 1;
                movel(tcp, cmd_mat);
                cmd_type = MoveL;
                memcpy(cmd_value, cmd, sizeof(double)*6);
                pthread_create(&move_wait_thread, NULL, move_wait_func, this);
            }
        }

        else{
            if(opt == "abs"){
                double cmd_mat[16] = {0,};
//                for(unsigned int i = 0; i < 3; i++){
//                    for(unsigned int j = 0; j < 3; j++){
//                        cmd_mat[i*4 + j] = cmd[3 + i*3 + j];
//                    }
//                }

//                cmd_mat[0] = cmd[3]; cmd_mat[1] = cmd[4]; cmd_mat[2] = cmd[5];
//                cmd_mat[4] = cmd[6]; cmd_mat[5] = cmd[7]; cmd_mat[6] = cmd[8];
//                cmd_mat[8] = cmd[9]; cmd_mat[9] = cmd[10]; cmd_mat[10] = cmd[11];
//                cmd_mat[0*4 + 3] = cmd[0];
//                cmd_mat[1*4 + 3] = cmd[1];
//                cmd_mat[2*4 + 3] = cmd[2];
//                cmd_mat[15] = 1;
				memcpy(cmd_mat, cmd, sizeof(double)*16);
                movel(base, cmd_mat);
                cmd_type = MoveL;
                memcpy(cmd_value, cmd, sizeof(double)*6);
                pthread_create(&move_wait_thread, NULL, move_wait_func, this);
            }

            if(opt == "rel"){
                for(unsigned int i = 0; i < 3; i++){
                    cmd[i] += pos[i];
                }
                double cmd_mat[16] = {0,};
                for(unsigned int i = 0; i < 3; i++){
                    for(unsigned int j = 0; j < 3; j++){
                        cmd_mat[i*4 + j] = mat[i*3 + j];
                    }
                }
                cmd_mat[0*4 + 3] = cmd[0];
                cmd_mat[1*4 + 3] = cmd[1];
                cmd_mat[2*4 + 3] = cmd[2];
                cmd_mat[15] = 1;
                movel(base, cmd_mat);
                cmd_type = MoveL;
                memcpy(cmd_value, cmd, sizeof(double)*6);
                pthread_create(&move_wait_thread, NULL, move_wait_func, this);
            }
        }
    }
}

void MainWindow::moveGripperOn()
{
    if(!cmd_flag){
//        gripper->gripper_grip(false);
        cmd_type = GripOn;
        usleep(10000);
        pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::moveGripperOff()
{
    if(!cmd_flag){
//        gripper->gripper_release(false);
        cmd_type = GripOff;
        usleep(10000);
        pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::moveGripperCustom()
{
    if(!cmd_flag){
//        gripper->gripper_release(false);
        cmd_type = GripCustom;
        usleep(10000);
        pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::moveChuckOpen()
{
	if(!cmd_flag){
		ControlBoxDigitalOut(8);
		usleep(1000000);
		ControlBoxDigitalOut(4);
		cmd_type = ChuckOpen;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}

}

void MainWindow::moveChuckClose()
{
	if(!cmd_flag){
		ControlBoxDigitalOut(8);
		usleep(1000000);
		ControlBoxDigitalOut(4);
		cmd_type = ChuckClose;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveDoorOpen()
{
	if(!cmd_flag){
		ControlBoxDigitalOut(2);
		cmd_type = DoorOpen;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveDoorClose()
{
	if(!cmd_flag){
		ControlBoxDigitalOut(1);
		cmd_type = DoorClose;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveLathe(){
	if(!cmd_flag){
		cmd_type = LathWait;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::btnRobotConnectClicked()
{
    if(ui->btnRobotConnect->text().compare("Connect")){
        RobotDisconnect();

        ui->btnRobotConnect->setText("Connect");

        robot_connected = false;
    }
    else{
        SetRobotConf(UR10, ui->txtRobotIP->text().toStdString().c_str(), ui->txtRobotPORT->text().toInt());
        RobotConnect();

        usleep(10000);

        ui->btnRobotConnect->setText("Disconnect");

        robot_connected = true;

//        usleep(1000000);

//        double q_goal[6] = {M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, 0};
//        movej(q_goal);
    }
}

void MainWindow::btnGripperConnectClicked()
{
    if(ui->btnGripperConnect->text().compare("Connect")){
//        gripperUpdateTimer->stop();
        gripper->disconnect();

        ui->btnGripperConnect->setText("Connect");

        gripper_connected = false;
    }
    else{
        gripper->connect(ui->txtGripperIP->text().toStdString(), ui->txtGripperPORT->text().toInt());

//        gripperUpdateTimer->start();

        usleep(10000);

        ui->btnGripperConnect->setText("Disconnect");

        gripper_connected = true;
    }
}

void MainWindow::btnInitClicked()
{
    mainState = Init;
    door_cnt = 0;
	obj_cnt = 0;

//	robot_connected = true;
//	gripper_connected = true;
}

void MainWindow::btnRunClicked()
{
    mainState = Start;
	pocState = Pick;
    door_cnt = 0;
    ui->rbRun->setChecked(true);
}

void MainWindow::btnPickObjClicked()
{
    mainState = Start;
    pocState = Pick;
    door_cnt = 0;
}

void MainWindow::btnChuckObjClicked()
{
    mainState = Start;
    pocState = Chuck1;
    door_cnt = 0;
}

void MainWindow::btnUnChuckObjClicked()
{
    mainState = Start;
    pocState = UnChuck1;
    door_cnt = 0;
}

void MainWindow::btnDoorSwitchClicked()
{
    mainState = Start;
    pocState = Door1;
    door_cnt = 0;
}

void MainWindow::btnPlaceObjClicked()
{
    mainState = Start;
    pocState = Place;
    door_cnt = 0;
}

void MainWindow::btnRePickObjClicked()
{
    mainState = Start;
    pocState = RePick;
    door_cnt = 0;
}

void MainWindow::btnChuckObj2Clicked()
{
    mainState = Start;
    pocState = Chuck2;
    door_cnt = 0;
}

void MainWindow::btnUnChuckObj2Clicked()
{
    mainState = Start;
    pocState = UnChuck2;
    door_cnt = 0;
}

void MainWindow::btnDoorSwitch2Clicked()
{
    mainState = Start;
    pocState = Door2;
    door_cnt = 0;
}

void MainWindow::btnRePlaceObjClicked()
{
    mainState = Start;
    pocState = RePlace;
    door_cnt = 0;
}

void MainWindow::btnTestClicked()
{
    mainState = TestStart;
    pocState = TestPick;
	obj_cnt = 0;
}

void MainWindow::btnLathStartClicked()
{
	mainState = Start;
	pocState = LatheStart1;
	obj_cnt = 0;
}

void MainWindow::btnLathWaitClicked()
{
	mainState = Start;
	pocState = LatheWait1;
	obj_cnt = 0;
}

void MainWindow::btnVisionListenClicked()
{
	tcpSocket->setPort(ui->txtVisionPORT->text().toInt());
	tcpSocket->start();

	usleep(1000000);

	system("gnome-terminal -- sh -c \"python3 /home/keti/Project/gaebong/poc_yolo/012cn.py\"");
}

void MainWindow::btnPrintClicked()
{
    for(unsigned int i = 0; i < 6; i++){
//        printf("%f\t", robotInfor.jnt[i]);
        std::cout << robotInfor.jnt[i] << ", ";
    }
//    printf("\n");
    std::cout << std::endl;
//    for(unsigned int i = 0; i < 3; i++){
////        printf("%f\t", robotInfor.mat[i]);
//        std::cout << robotInfor.mat[4*i + 3] << ", ";
//    }
//    std::cout << std::endl;
	for(unsigned int i = 0; i < 4; i++){
		for(unsigned int j = 0; j < 4; j++){
            std::cout << robotInfor.mat[i*4 + j] << ",";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    //    printf("\n");
}

void MainWindow::btnGripperInitClicked()
{
    gripper->gripper_init();
    gripper_init = true;
}

void MainWindow::btnGripperGripClicked()
{
    gripper->gripper_grip();
}

void MainWindow::btnGripperReleaseClicked()
{
    gripper->gripper_release();
}

void MainWindow::robotStateUpdate()
{
    robotInfor = RobotInfo();

    state = robotInfor.state;
    ui->txtRobotStatus->setText(QString::number(state));

    if(state == 2.0) {
        moving = true;
    }
	else{
		moving = false;
	}

    for(unsigned int i = 0; i < 6; i++){
        txtJoint[i]->setText(QString::number(robotInfor.jnt[i]));
    }
    for(unsigned int i = 0; i < 3; i++){
        txtPose[i]->setText(QString::number(robotInfor.mat[i*4 + 3]));
    }

    for(unsigned int i = 0; i < 6; i++){
        jnt[i] = robotInfor.jnt[i];
    }

    for(unsigned int i = 0; i < 3; i++){
        for(unsigned int j = 0; j < 3; j++){
            mat[i*3 + j] = robotInfor.mat[i*4 + j];
        }
    }
    for(unsigned int i = 0; i < 3; i++){
        pos[i] = robotInfor.mat[i*4 + 3];
    }

//    printf("\n%f, %f, %f, %f\n", robotInfor.mat[0], robotInfor.mat[1], robotInfor.mat[2], robotInfor.mat[3]);
//    printf("%f, %f, %f, %f\n", robotInfor.mat[4], robotInfor.mat[5], robotInfor.mat[6], robotInfor.mat[7]);
//    printf("%f, %f, %f, %f\n", robotInfor.mat[8], robotInfor.mat[9], robotInfor.mat[10], robotInfor.mat[11]);
//    printf("%f, %f, %f, %f\n\n", robotInfor.mat[12], robotInfor.mat[13], robotInfor.mat[14], robotInfor.mat[15]);

    double ang_x, ang_y, ang_z;
    Conf.InverseRot(mat, &ang_x,&ang_y,&ang_z);

    txtPose[3]->setText(QString::number(ang_x));
    txtPose[4]->setText(QString::number(ang_y));
    txtPose[5]->setText(QString::number(ang_z));
}

void MainWindow::gripperStateUpdate()
{
    gripper->get_write_reg(gripper_write_reg);
    gripper->get_read_reg(gripper_read_reg);

    ui->txtBasePosition->setText(QString::number(gripper_write_reg[4]));
    ui->txtShiftPosition->setText(QString::number(gripper_write_reg[5]));
    ui->txtWorkPosition->setText(QString::number(gripper_write_reg[7]));
    ui->txtCurrentPosition->setText(QString::number(gripper_read_reg[2]));

    ui->txtErrorNum->setText(QString::number(((gripper_read_reg[0]&0x0004) == 0x0004)) + "(" + QString::number(gripper_read_reg[1], 16) + ")");
//    ui->txtErrorNum->setText(QString::number(((gripper_read_reg[0]&0x0004) == 0x0004)) + "(" + QString::number((gripper_read_reg[0]&0x0008) == 0x0008) + ")");

//    if(((gripper_read_reg[0]&0x0004) == 0x0004)){
//        gripper->gripper_init();
//    }
}

void MainWindow::visionStateUpdate(){
	tcpSocket->getRecvData(visionData);
//	std::cout << "visionData : " << (int)visionData[0] << ", " << (int)visionData[1] << ", " << (int)visionData[2] << std::endl;

	memcpy(&mchState, visionData, sizeof(unsigned char)*3);
//	std::cout << "mchState : " << (int)mchState.door << ", " << (int)mchState.chuck << ", " << (int)mchState.working << std::endl;

	ui->txtStateDoor->setText(mchState.door == mchState.DoorOpen ? "Open" : mchState.door == mchState.DoorClose ? "Close" : "- - -");
	ui->txtStateChuck->setText(mchState.chuck == mchState.ChuckClose ? "Close" : mchState.chuck == mchState.ChuckOpen ? "Open" : "- - -");
	ui->txtStateWork->setText(mchState.work == mchState.WorkComplete ? "Complete" : mchState.work == mchState.Working ? "Working..." : "- - -");


}

void* MainWindow::move_wait_func(void *arg){
    MainWindow *pThis = static_cast<MainWindow*>(arg);

    pThis->move_complete = false;
    pThis->cmd_flag = true;

    unsigned int cnt = 0;
    bool run = true;
	while(run){
        switch(pThis->cmd_type){
            case MoveJ:
            {
				double err_max = abs(pThis->cmd_value[0] - pThis->jnt[0]);
				double err = 0;
				for(unsigned int i = 1; i < 6; i++){
					err = abs(pThis->cmd_value[i] - pThis->jnt[i]);
					err_max = err > err_max ? err : err_max;
				}
				if(pThis->moving && pThis->state == 1.0) {
					std::cout << "move finish" << std::endl;
					pThis->moving = false;
					run = false;
					break;
				}
				if(cnt >= 300 && pThis->state == 1.0) {
					std::cout << "move wait timeout " << cnt << std::endl;
					run = false;
					break;
				}
				if(err_max < 1e-3) {
					std::cout << "goal reach" << std::endl;
					run = false;
					break;
				}
				cnt++;

                break;
            }
            case MoveL:
            {
//				double err_max = abs(pThis->cmd_value[0] - pThis->pos[0]);
//				double err = 0;
//				for(unsigned int i = 1; i < 3; i++){
//					err = abs(pThis->cmd_value[i] - pThis->pos[i]);
//					err_max = err > err_max ? err : err_max;
//				}
//				std::cout << err_max << std::endl;
				if(pThis->moving && pThis->state == 1.0) {
					std::cout << "move finish" << std::endl;
					pThis->moving = false;
					run = false;
					break;
				}
				if(cnt >= 300 && pThis->state == 1.0) {
					std::cout << "move wait timeout " << cnt << std::endl;
					run = false;
					break;
				}
//				if(err_max < 1e-3) {
//					std::cout << "goal reach" << std::endl;
//					run = false;
//				}
				cnt++;
                break;
            }
            case GripOn:
            {
                pThis->gripper->gripper_grip();
                pThis->moving = false;
				run = false;
                break;
            }
            case GripOff:
            {
                pThis->gripper->gripper_release();
                pThis->moving = false;
				run = false;
                break;
            }
            case GripCustom:
            {
                pThis->gripper->gripper_custom(1100, 50, 50);
                pThis->moving = false;
				run = false;
                break;
            }
            case ChuckOpen:
			{
				if(pThis->mchState.chuck == pThis->mchState.ChuckOpen){
					usleep(1000000);
					ControlBoxDigitalOut(8);
					usleep(1000000);
					ControlBoxDigitalOut(0);
					run = false;
				}
                break;
            }
            case ChuckClose:
            {
				if(pThis->mchState.chuck == pThis->mchState.ChuckClose){
					usleep(1000000);
					ControlBoxDigitalOut(8);
					usleep(1000000);
					ControlBoxDigitalOut(0);
					run = false;
				}
                break;
            }
            case DoorOpen:
            {
				if(pThis->mchState.door == pThis->mchState.DoorOpen)
				{
					ControlBoxDigitalOut(0);
                    run = false;
                }
                break;
            }
            case DoorClose:
            {
				if(pThis->mchState.door == pThis->mchState.DoorClose)
				{
					ControlBoxDigitalOut(0);
                    run = false;
                }
                break;
            }
			case LathWait:
			{
				if(pThis->mchState.work == pThis->mchState.WorkComplete)
				{
					run = false;
				}
				break;
			}
            default:
            {
                break;
            }
        }

        usleep(2000);
    }

    pThis->mainState = Wait;
    pThis->move_complete = true;
    pThis->moving = false;
    pThis->cmd_flag = false;

    std::cout << "\nfinish move wait thread\n" << std::endl;

    return nullptr;
}

void MainWindow::POCTestFunc()
{
    switch(pocState){
        case TestPick:
        {
            if(POCTestPick()){
                pocState = TestPlace;
                pocSubState = 0;
            }
            break;
        }
//        case TestPlace:
//        {
//            if(POCTestPlace()){
//                obj_cnt++;
//                pocState = obj_cnt < 12 ? TestPick : TestFinish;
//                pocSubState = 0;
//            }
//            break;
//        }
//        case TestFinish:
//        {
//            POCReady();
//            pocSubState = 0;
//            break;
//        }
        default:
        {
            break;
        }
    }
}

bool MainWindow::POCTestPick()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
//                moveJoint(TJS_ready2pick2, duration_fast);
                break;
            }
            case 1:
            {
//                moveJoint(TJS_ready2pick3, duration_slow);
                break;
            }
            case 2:
            {
                moveGripperOff();
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, 0, 0, 0, 0};
                offset[0] = -TPick_obj_offset_x*(obj_cnt%3);
                offset[2] = TPick_obj_offset_y*(obj_cnt/3);
//                movePose(offset, duration_fast, "rel", "tcp");
                break;
            }
            case 4:
            {
//                double offset[6] = {0, 0, TWS_to_obj_z, 0, 0, 0};
//                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 5:
            {
                moveGripperOn();
                break;
            }
            case 6:
            {
                double offset[6] = {0, 0, -TWS_to_obj_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
//            case 6:
//            {
//                moveJoint(TJS_withdraw_pick, duration_fast);
//                break;
//            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCTestPlace()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(TJS_withdraw_pick, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(TJS_over_place, duration_fast);
                break;
            }
            case 2:
            {
                double offset[6] = {0, 0, 0, 0, 0, 0};
                offset[0] = TPlace_obj_offset_x*(obj_cnt%3);
                offset[2] = -TPlace_obj_offset_y*(obj_cnt/3);
                movePose(offset, duration_fast, "rel", "tcp");
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, TWS_place_z, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 4:
            {
                moveGripperOff();
                break;
            }
            case 5:
            {
                double offset[6] = {0, 0, -TWS_place_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}
