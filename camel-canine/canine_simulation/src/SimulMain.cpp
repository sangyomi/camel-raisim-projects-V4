//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulMain.hpp>

#define SLOW_MOTION         1

pthread_t RTThreadControllerHigh;
pthread_t RTThreadControllerLow;
pthread_t NRTThreadCommand;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;
pCUSTOM_DATA sharedCustom;
pAXIS joystickAxis;
pBUTTON joystickButton;

raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"/canine/canineV4/urdf/canineV4_2.urdf");

RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR) + "canine/canineV4/urdf/canineV4.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);

SimulCommand userCommand;
SimulVisualizer Visualizer(&world, robot, &server);
SimulControlPanel ControlPanel(&world, robot);
SimulControlPanelHigh ControlPanelHigh;
SimulStateEstimator StateEstimator(robot,model);

///for contact
auto footIndexFR = robot->getBodyIdx("FR_calf");
auto footIndexFL = robot->getBodyIdx("FL_calf");
auto footIndexRR = robot->getBodyIdx("HR_calf");
auto footIndexRL = robot->getBodyIdx("HL_calf");

void* RTControllerThreadLow(void* arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(LOW_CONTROL_dT * SLOW_MOTION * 1e6);
    std::cout << "[MAIN FSM] Generated Low Controller RT Thread : " << 1 / double(LOW_CONTROL_dT) << " Hz" <<std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            ControlPanel.ControllerFunction();
            StateEstimator.StateEstimatorFunction();
        }

        for(auto& contact : robot->getContacts())
        {
            if(contact.skip()) continue;

            if (footIndexFL == contact.getlocalBodyIndex()){
                sharedMemory->simulContactForceFL = sqrt(pow(contact.getImpulse().e()[0],2)
                                                         + pow(contact.getImpulse().e()[1],2)
                                                         + pow(contact.getImpulse().e()[2],2))/LOW_CONTROL_dT ;
            }
            if (footIndexFR == contact.getlocalBodyIndex()){
                sharedMemory->simulContactForceFR = sqrt(pow(contact.getImpulse().e()[0],2)
                                                         + pow(contact.getImpulse().e()[1],2)
                                                         + pow(contact.getImpulse().e()[2],2))/LOW_CONTROL_dT ;
            }
            if (footIndexRL == contact.getlocalBodyIndex()){
                sharedMemory->simulContactForceHL = sqrt(pow(contact.getImpulse().e()[0],2)
                                                         + pow(contact.getImpulse().e()[1],2)
                                                         + pow(contact.getImpulse().e()[2],2))/LOW_CONTROL_dT ;
            }
            if (footIndexRR == contact.getlocalBodyIndex()){
                sharedMemory->simulContactForceHR = sqrt(pow(contact.getImpulse().e()[0],2)
                                                         + pow(contact.getImpulse().e()[1],2)
                                                         + pow(contact.getImpulse().e()[2],2))/LOW_CONTROL_dT ;
            }

        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, Low Controller RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void* RTControllerThreadHigh(void* arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(HIGH_CONTROL_dT * SLOW_MOTION * 1e6); // 200Hz 짜리 쓰레드
    std::cout << "[MAIN FSM] Generated High Controller RT Thread : " << 1 / double(HIGH_CONTROL_dT) << " Hz" <<std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            ControlPanelHigh.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, High Controller RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* NRTCommandThread(void* arg)
{
    std::cout << "[MAIN FSM] Generated Command NRT Thread : " << 1 / double(CMD_dT) << " Hz" <<std::endl;
    while (true)
    {
        userCommand.commandFunction();
        Visualizer.UpdateFootStep();
        usleep(CMD_dT * 1e6);
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

void StartSimulation()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    sharedCustom = (pCUSTOM_DATA)malloc(sizeof(CUSTOM_DATA));
    joystickAxis = (pAXIS)malloc(sizeof(AXIS));
    joystickButton = (pBUTTON)malloc(sizeof(BUTTON));

    clearSharedMemory();
    clearJoystickInfo();

    server.launchServer(8080);

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_rt1 = generate_rt_thread(RTThreadControllerLow, RTControllerThreadLow, "rt_thread1", 5, 0, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadControllerHigh, RTControllerThreadHigh, "rt_thread2", 7, 0, NULL);

}