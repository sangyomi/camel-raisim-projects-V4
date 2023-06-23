//
// Created by hs on 22. 10. 5.
//

#include <canine_fsm/MainFSM.hpp>

pthread_t RTThreadControllerHigh;
pthread_t RTThreadControllerLow;
pthread_t RTThreadIMU;
pthread_t NRTThreadCommand;
pthread_t NRTThreadVisual;
pthread_t NRTThreadCANLF;
pthread_t NRTThreadCANRF;
pthread_t NRTThreadCANLB;
pthread_t NRTThreadCANRB;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;
pCUSTOM_DATA sharedCustom;
pAXIS joystickAxis;
pBUTTON joystickButton;

CanMotorLF canLF("can20");
CanMotorRF canRF("can19");
CanMotorLB canLB("can18");
CanMotorRB canRB("can21");

RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR) + "canine/canineV4/urdf/canineV4_2.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);

DataAnalysis SaveStates;
Command userCommand;
raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"canine/canineV4/urdf/canineV4_with_head.urdf");
RobotVisualization userVisual(&world, robot, &server);
StateEstimator robotstate(model);
LowControlMain LowController;
HighControlMain HighController;

const std::string mComPort = "/dev/ttyACM0";
const mscl::Connection mConnection = mscl::Connection::Serial(mComPort);
mscl::InertialNode node(mConnection);
LordImu3DmGx5Ahrs IMUBase(&node);

void *RTControllerThreadHigh(void *arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(HIGH_CONTROL_dT * 1e6);
    std::cout << "[MAIN FSM] Generated High Controller RT Thread : " << 1 / double(PERIOD_US) * 1e6 << " Hz" <<std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            HighController.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, High Controller RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void *RTControllerThreadLow(void *arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(LOW_CONTROL_dT * 1e6);
    std::cout << "[MAIN FSM] Generated Low Controller RT Thread : " << 1 / double(PERIOD_US) * 1e6 << " Hz" <<std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            robotstate.StateEstimatorFunction();
            LowController.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, Low Controller RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* NRTCommandThread(void* arg)
{
    std::cout << "[MAIN FSM] Generated Command NRT Thread : " << 1 / double(CMD_dT) << " Hz" <<std::endl;
    while (true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT * 1e6);
    }
}

void* NRTVisualThread(void* arg)
{
    std::cout << "[MAIN FSM] Generated Visual NRT Thread : " << 1 / double(VISUAL_dT) << " Hz" <<std::endl;
    while (true)
    {
        userVisual.VisualFunction();
        usleep(VISUAL_dT * 1e6);
    }
}

void* RTImuThread(void* arg)
{
    struct timespec TIME_1;
    struct timespec TIME_2;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(IMU_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "[MAIN FSM] Generated IMU RT Thread : " << 1 / double(PERIOD_US) << " Hz" <<std::endl;

    IMUBase.GetCurrentConfig(node);
    IMUBase.SetConfig(1000);
    double* baseEulerAngle;
    double* baseAngularVelocity;
    double* baseLinearAcceleration;
    double yawOffset = 0.0;
    node.setSensorToVehicleRotation_eulerAngles({0.0,0.0,0.0});
//    IMUBase.GetCurrentConfig(node);
    bool isFirstRun = true;
    double tempYaw;
    int multiTurn = 0;
    double baseSingleTurnEulerAngle[3];
    Eigen::Quaternion<double> quaternion;
    Eigen::Vector3d eulerEigen;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_gettime(CLOCK_REALTIME,&TIME_1);
        IMUBase.ParseData();
        baseAngularVelocity = IMUBase.GetAngularVelocity();
        baseEulerAngle = IMUBase.GetEulerAngle();
        baseLinearAcceleration = IMUBase.GetLinearAcceleration();
        if(isFirstRun)
        {
            yawOffset = -baseEulerAngle[2];
            isFirstRun = false;
        }
        else
        {
            sharedMemory->baseEulerVelocity[0] = baseAngularVelocity[0];
            sharedMemory->baseEulerVelocity[1] = -baseAngularVelocity[1];
            sharedMemory->baseEulerVelocity[2] = -baseAngularVelocity[2];

            baseSingleTurnEulerAngle[0] = baseEulerAngle[0];
            baseSingleTurnEulerAngle[1] = -baseEulerAngle[1];
            baseSingleTurnEulerAngle[2] = -baseEulerAngle[2]-yawOffset;

            if(tempYaw > 3.1 && -baseEulerAngle[2] < 3.1/2.0)
            {
                multiTurn++;
            }
            if(tempYaw < -3.1 && -baseEulerAngle[2] > -3.1/2.0)
            {
                multiTurn--;
            }

            sharedMemory->baseAcceleration[0] = baseLinearAcceleration[0];
            sharedMemory->baseAcceleration[1] = -baseLinearAcceleration[1];
            sharedMemory->baseAcceleration[2] = -baseLinearAcceleration[2];

            eulerEigen << baseSingleTurnEulerAngle[0], baseSingleTurnEulerAngle[1], baseSingleTurnEulerAngle[2];
            quaternion = Eigen::AngleAxisd(eulerEigen[0], Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(eulerEigen[1], Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(eulerEigen[2], Eigen::Vector3d::UnitZ());

            sharedMemory->baseQuartPosition[0] = quaternion.w();
            sharedMemory->baseQuartPosition[1] = quaternion.x();
            sharedMemory->baseQuartPosition[2] = quaternion.y();
            sharedMemory->baseQuartPosition[3] = quaternion.z();

            sharedMemory->baseEulerPosition[0] = baseSingleTurnEulerAngle[0];
            sharedMemory->baseEulerPosition[1] = baseSingleTurnEulerAngle[1];
            sharedMemory->baseEulerPosition[2] = baseSingleTurnEulerAngle[2] + multiTurn * 2.0 * 3.141592;
        }
        tempYaw = -baseEulerAngle[2];

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);

//        std::cout<<"[IMU thread] imu : "<<baseAngularVelocity[0]<<", "<<-baseAngularVelocity[1]<<", "<<-baseAngularVelocity[2]<<std::endl;
//        std::cout<<"[NRT IMU THREAD] local time : "<<sharedMemory->localTime<<std::endl;
//        std::cout<<"[NRT IMU THREAD] imu : "<<sharedMemory->baseEulerPosition[0]<<", "<<sharedMemory->baseEulerPosition[1]<<", "<<sharedMemory->baseEulerPosition[2]<<std::endl;
//        std::cout<<"[NRT IMU THREAD] imu : "<<sharedMemory->baseAcceleration[0]<<", "<<sharedMemory->baseAcceleration[1]<<", "<<sharedMemory->baseAcceleration[2]<<std::endl;
//        std::cout<< std::endl;
//        clock_gettime(CLOCK_REALTIME,&TIME_2);
//        sharedMemory->solvedGRF[0][0] = timediff_us(&TIME_1, &TIME_2) * 1e-3;

        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, IMU RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* NRTCANLF(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-LF NRT Thread : " << "non-sleep loop" <<std::endl;

    while (true)
    {
        canLF.CanFunction();
    }
}

void* NRTCANRF(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-RF NRT Thread : " << "non-sleep loop" <<std::endl;

    while (true)
    {
        canRF.CanFunction();
    }
}

void* NRTCANLB(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-LB NRT Thread : " << "non-sleep loop" <<std::endl;

    while (true)
    {
        canLB.CanFunction();
    }
}

void* NRTCANRB(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-RB NRT Thread : " << "non-sleep loop" <<std::endl;

    while (true)
    {
        canRB.CanFunction();
    }
}

void clearSharedMemory()
{
    for (int i =0 ; i<MPC_HORIZON*4;i++)
    {
        sharedMemory->gaitTable[i] = 1;
    }

    sharedMemory->gaitState = STAND;
    sharedMemory->gaitChangeFlag = false;
    sharedMemory->throwFlag = false;

    sharedMemory->isNan = false;
    sharedMemory->isRamp = false;
    sharedMemory->bIsEndHome = false;
    sharedMemory->newCommand = false;

    sharedMemory->canLFStatus = false;
    sharedMemory->canRFStatus = false;
    sharedMemory->canLBStatus = false;
    sharedMemory->canRBStatus = false;

    sharedMemory->motorStatus = false;
    sharedMemory->motorLFState = false;
    sharedMemory->motorRFState = false;
    sharedMemory->motorLBState = false;
    sharedMemory->motorRBState = false;

    sharedMemory->FSMState = FSM_INITIAL;
    sharedMemory->LowControlState = STATE_LOW_CONTROL_STOP;
    sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;

    sharedMemory->canLFState = CAN_NO_ACT;
    sharedMemory->canRFState = CAN_NO_ACT;
    sharedMemory->canLBState = CAN_NO_ACT;
    sharedMemory->canRBState = CAN_NO_ACT;

    sharedMemory->localTime = 0;

    for (int index = 0; index < MOTOR_NUM; index++)
    {
        sharedMemory->motorErrorStatus[index] = 0;
        sharedMemory->motorTemp[index] = 0;
        sharedMemory->motorVoltage[index] = 0;
        sharedMemory->motorPosition[index] = 0;
        sharedMemory->motorVelocity[index] = 0;
        sharedMemory->motorTorque[index] = 0;
        sharedMemory->motorDesiredPosition[index] = 0;
        sharedMemory->motorDesiredVelocity[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
        sharedMemory->motorPrevDesiredPosition[index] = 0;
    }

    sharedMemory->basePosition.setZero();
    sharedMemory->baseVelocity.setZero();
    sharedMemory->baseQuartPosition[0] = 1.0;
    sharedMemory->baseQuartPosition[1] = 0.0;
    sharedMemory->baseQuartPosition[2] = 0.0;
    sharedMemory->baseQuartPosition[3] = 0.0;
    sharedMemory->baseDesiredPosition.setZero();
    sharedMemory->baseDesiredVelocity.setZero();
    sharedMemory->baseDesiredQuartPosition[0] = 1.0;
    sharedMemory->baseDesiredQuartPosition[1] = 0.0;
    sharedMemory->baseDesiredQuartPosition[2] = 0.0;
    sharedMemory->baseDesiredQuartPosition[3] = 0.0;
    sharedMemory->baseDesiredEulerPosition.setZero();
    sharedMemory->baseDesiredEulerVelocity.setZero();
    sharedMemory->baseLocalDesiredVelocity.setZero();
    for (int index = 0; index < 3; index++)
    {
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
        sharedMemory->baseAcceleration[index] = 0;
        sharedMemory->testBasePos[index] = 0;
        sharedMemory->testBaseVel[index] = 0;
    }

    for (int index = 0; index < 4; index++)
    {
        sharedMemory->pdTorque[index].setZero();
        sharedMemory->mpcTorque[index].setZero();
        sharedMemory->bodyFootPosition[index].setZero();
        sharedMemory->globalFootPosition[index].setZero();
        sharedMemory->desiredFootPosition[index].setZero();
        sharedMemory->visualPosition[index].setZero();
        sharedMemory->contactState[index] = true;
        sharedMemory->tempRes[index] = 0;
        sharedMemory->solvedGRF[index].setZero();
    }
}

void clearJoystickInfo()
{
    joystickAxis->LeftStickX = 0;
    joystickAxis->LeftStickY = 0;
    joystickAxis->LeftTrigger = -30767;
    joystickAxis->RightStickX = 0;
    joystickAxis->RightStickY = 0;
    joystickAxis->RightTrigger = -30767;
    joystickAxis->DpadX = 0;
    joystickAxis->DpadY = 0;

    joystickButton->FaceButtonA = 0;
    joystickButton->FaceButtonB = 0;
    joystickButton->FaceButtonX = 0;
    joystickButton->FaceButtonY = 0;
    joystickButton->LeftBumper = 0;
    joystickButton->RightBumper = 0;
    joystickButton->Back = 0;
    joystickButton->Start = 0;
    joystickButton->Guide = 0;
    joystickButton->LeftStick = 0;
    joystickButton->RightStick = 0;
}

void StartFSM()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    sharedCustom = (pCUSTOM_DATA)malloc(sizeof(CUSTOM_DATA));
    joystickAxis = (pAXIS)malloc(sizeof(AXIS));
    joystickButton = (pBUTTON)malloc(sizeof(BUTTON));

    clearSharedMemory();
    clearJoystickInfo();

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt2 = generate_nrt_thread(NRTThreadVisual, NRTVisualThread, "nrt_thread2", 1, NULL);
    int thread_id_nrt3 = generate_nrt_thread(NRTThreadCANLF, NRTCANLF, "nrt_thread3", 4, NULL);
    int thread_id_nrt4 = generate_nrt_thread(NRTThreadCANRF, NRTCANRF, "nrt_thread4", 4, NULL);
    int thread_id_nrt5 = generate_nrt_thread(NRTThreadCANLB, NRTCANLB, "nrt_thread5", 4, NULL);
    int thread_id_nrt6 = generate_nrt_thread(NRTThreadCANRB, NRTCANRB, "nrt_thread6", 4, NULL);

    int thread_id_rt1 = generate_rt_thread(RTThreadControllerHigh, RTControllerThreadHigh, "rt_thread1", 5, 99, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadControllerLow, RTControllerThreadLow, "rt_thread2", 6, 99, NULL);
    int thread_id_rt3 = generate_rt_thread(RTThreadIMU, RTImuThread, "rt_thread3", 7, 99, NULL);
}