#include <canine_util/Command.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;
extern pAXIS joystickAxis;
extern pBUTTON joystickButton;

CanineFilter::MAF MAFbaseLocalDesiredVel0(500);
CanineFilter::MAF MAFbaseDesiredVel0(500);
CanineFilter::MAF MAFbaseDesiredVel1(500);
CanineFilter::MAF MAFbaseDesiredEulerVelocity(500);

Command::Command()
: mLocalVelocity(0.0)
, mDeadBand(2000)
{
//    initializeJoystick();
    initializeJoystickOnex();
    mBaseReferenceEulerPosition[0] = 0.0;
    mBaseReferenceEulerPosition[1] = 0.0;
    mBaseReferenceEulerPosition[2] = 0.0;
}

void Command::commandFunction()
{
//    readJoystick();
    readJoystickOnex();
    mappingJoystickCommand();

    if(sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch(incomingCommand)
        {
            case CAN_ON:
            {
                sharedMemory->canLFState = CAN_INIT;
                sharedMemory->canRFState = CAN_INIT;
                sharedMemory->canLBState = CAN_INIT;
                sharedMemory->canRBState = CAN_INIT;
                sleep(1);
                sharedMemory->newCommand = true;
                sharedCommand->userCommand = VISUAL_ON;
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                sharedMemory->newCommand = true;
                sharedCommand->userCommand = MOTOR_ON;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->canLFState = CAN_MOTOR_ON;
                sharedMemory->canRFState = CAN_MOTOR_ON;
                sharedMemory->canLBState = CAN_MOTOR_ON;
                sharedMemory->canRBState = CAN_MOTOR_ON;
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->canLFState = CAN_MOTOR_OFF;
                sharedMemory->canRFState = CAN_MOTOR_OFF;
                sharedMemory->canLBState = CAN_MOTOR_OFF;
                sharedMemory->canRBState = CAN_MOTOR_OFF;
                break;
            }
            case CHANGE_GAIT_STAND:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_STAND;
                break;
            }
            case CHANGE_GAIT_TROT_SLOW:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_TROT_SLOW;
                break;
            }
            case CHANGE_GAIT_TROT_FAST:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_TROT_FAST;
                break;
            }
            case CHANGE_GAIT_OVERLAP_TROT_FAST:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_OVERLAP_TROT_FAST;
                break;
            }
            case HOME:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_UP_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_UP_READY;
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            case READY:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_DOWN_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_DOWN_READY;
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            case CUSTOM_1:
            {
                break;
            }
            case CUSTOM_2:
            {
                for(int i = 0 ; i < MOTOR_NUM ; i++)
                {
                    sharedMemory->motorDesiredTorque[i] = 0.0;
                }
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            default:
                break;
        }
    }
}

void Command::initializeJoystick()
{
    const char* joyStickFilePath = "/dev/input/js0";
    mJoystickFd = -1;
    mJoystickNumOfAxis = 0;
    mJoystickNumOfButton = 0;
    if ((mJoystickFd = open(joyStickFilePath, O_RDONLY)) < 0)
    {
        std::cerr << "Failed to open " << joyStickFilePath << std::endl;
        return;
    }
    ioctl(mJoystickFd, JSIOCGAXES, &mJoystickNumOfAxis);
    ioctl(mJoystickFd, JSIOCGBUTTONS, &mJoystickNumOfButton);
    ioctl(mJoystickFd, JSIOCGNAME(80), &mJoystickName);
    mJoystickButton.resize(mJoystickNumOfButton, 0);
    mJoystickAxis.resize(mJoystickNumOfAxis, 0);
    fcntl(mJoystickFd, F_SETFL, O_NONBLOCK);   // using non-blocking mode

//    char dualshockFirstWord[2] = {"S"};
//    if(mJoystickName[0] == dualshockFirstWord[0])
//    {
//        std::cout << "[COMMAND] Joystick: Dual Shock Controller" << std::endl;
//        mJoystickType = DUAL_SHOCK;
//    }
//    else
//    {
//        std::cout << "[COMMAND] Joystick: XBox Controller" << std::endl;
//        mJoystickType = XBOX_CONTROLLER;
//    }

    std::cout << "[COMMAND] Joystick: Dual Shock Controller" << std::endl;
    mJoystickType = DUAL_SHOCK;
}

void Command::initializeJoystickOnex()
{
    mJoystickButton.resize(11, 0);
    mJoystickAxis.resize(8, 0);
    std::cout << "[COMMAND] Joystick: XBox Controller" << std::endl;
    mJoystickType = XBOX_CONTROLLER;
}

void Command::readJoystick()
{
    js_event js;
    read(mJoystickFd, &js, sizeof(js_event));
    switch (js.type & ~JS_EVENT_INIT)
    {
    case JS_EVENT_AXIS:
        if(((int)js.value > mDeadBand)||((int)js.value < -mDeadBand))
        {
            int sign;
            if((int)js.value > 0)
            {
                sign = 1;
            }
            else
            {
                sign = -1;
            }
            mJoystickAxis[(int)js.number] = js.value - sign * mDeadBand;
        }
        else
        {
            mJoystickAxis[(int)js.number] = 0;
        }
        break;
    case JS_EVENT_BUTTON:
        mJoystickButton[(int)js.number] = js.value;
        break;
    }

    if(mJoystickAxis[7] > 0)
    {
        mJoystickLeftButtons[0] = true;
    }
    else
    {
        mJoystickLeftButtons[0] = false;
    }

    if(mJoystickAxis[6] > 0)
    {
        mJoystickLeftButtons[1] = true;
    }
    else
    {
        mJoystickLeftButtons[1] = false;
    }

    if(mJoystickAxis[6] < 0)
    {
        mJoystickLeftButtons[2] = true;
    }
    else
    {
        mJoystickLeftButtons[2] = false;
    }

    if(mJoystickAxis[7] < 0)
    {
        mJoystickLeftButtons[3] = true;
    }
    else
    {
        mJoystickLeftButtons[3] = false;
    }

    mJoystickLeftAxis[0] = (double)mJoystickAxis[0];
    mJoystickLeftAxis[1] = (double)mJoystickAxis[1];
    mJoystickRightAxis[0] = (double)mJoystickAxis[3];
    mJoystickRightAxis[1] = (double)mJoystickAxis[4];

    switch(mJoystickType)
    {
        case DUAL_SHOCK:
        {
            mJoystickRightButtons[0] = (bool)mJoystickButton[0];
            mJoystickRightButtons[1] = (bool)mJoystickButton[1];
            mJoystickRightButtons[2] = (bool)mJoystickButton[3];
            mJoystickRightButtons[3] = (bool)mJoystickButton[2];

            mJoystickRearButtons[0] = (bool)mJoystickButton[4];
            mJoystickRearButtons[1] = (bool)mJoystickButton[5];
            mJoystickRearButtons[2] = (bool)mJoystickButton[6];
            mJoystickRearButtons[3] = (bool)mJoystickButton[7];

            mJoystickFunctionButtons[0] = (bool)mJoystickButton[8];
            mJoystickFunctionButtons[1] = (bool)mJoystickButton[9];
            break;
        }
        case XBOX_CONTROLLER:
        {
            mJoystickRightButtons[0] = (bool)mJoystickButton[0];
            mJoystickRightButtons[1] = (bool)mJoystickButton[1];
            mJoystickRightButtons[2] = (bool)mJoystickButton[2];
            mJoystickRightButtons[3] = (bool)mJoystickButton[3];

            mJoystickRearButtons[0] = (bool)mJoystickButton[4];
            mJoystickRearButtons[1] = (bool)mJoystickButton[5];
            if(mJoystickAxis[2] > 0)
            {
                mJoystickRearButtons[2] = true;
            }
            else
            {
                mJoystickRearButtons[2] = false;
            }
            if(mJoystickAxis[5] > 0)
            {
                mJoystickRearButtons[3] = true;
            }
            else
            {
                mJoystickRearButtons[3] = false;
            }

            mJoystickFunctionButtons[0] = (bool)mJoystickButton[6];
            mJoystickFunctionButtons[1] = (bool)mJoystickButton[7];
            break;
        }
    }
}

void Command::readJoystickOnex()
{
    mJoystickAxis[0] = joystickAxis->LeftStickX;
    mJoystickAxis[1] = joystickAxis->LeftStickY;
    mJoystickAxis[2] = joystickAxis->LeftTrigger;
    mJoystickAxis[3] = joystickAxis->RightStickX;
    mJoystickAxis[4] = joystickAxis->RightStickY;
    mJoystickAxis[5] = joystickAxis->RightTrigger;
    mJoystickAxis[6] = joystickAxis->DpadX;
    mJoystickAxis[7] = joystickAxis->DpadY;
    mJoystickButton[0] = static_cast<char>(joystickButton->FaceButtonA);
    mJoystickButton[1] = static_cast<char>(joystickButton->FaceButtonB);
    mJoystickButton[2] = static_cast<char>(joystickButton->FaceButtonX);
    mJoystickButton[3] = static_cast<char>(joystickButton->FaceButtonY);
    mJoystickButton[4] = static_cast<char>(joystickButton->LeftBumper);
    mJoystickButton[5] = static_cast<char>(joystickButton->RightBumper);
    mJoystickButton[6] = static_cast<char>(joystickButton->Back);
    mJoystickButton[7] = static_cast<char>(joystickButton->Start);
    mJoystickButton[8] = static_cast<char>(joystickButton->Guide);
    mJoystickButton[9] = static_cast<char>(joystickButton->LeftStick);
    mJoystickButton[10] = static_cast<char>(joystickButton->RightStick);

    if(mJoystickAxis[7] > 0)
    {
        mJoystickLeftButtons[0] = true;
    }
    else
    {
        mJoystickLeftButtons[0] = false;
    }

    if(mJoystickAxis[6] > 0)
    {
        mJoystickLeftButtons[1] = true;
    }
    else
    {
        mJoystickLeftButtons[1] = false;
    }

    if(mJoystickAxis[6] < 0)
    {
        mJoystickLeftButtons[2] = true;
    }
    else
    {
        mJoystickLeftButtons[2] = false;
    }

    if(mJoystickAxis[7] < 0)
    {
        mJoystickLeftButtons[3] = true;
    }
    else
    {
        mJoystickLeftButtons[3] = false;
    }

    mJoystickLeftAxis[0] = (double)mJoystickAxis[0];
    mJoystickLeftAxis[1] = (double)mJoystickAxis[1];
    mJoystickRightAxis[0] = (double)mJoystickAxis[3];
    mJoystickRightAxis[1] = (double)mJoystickAxis[4];

    switch(mJoystickType)
    {
        case DUAL_SHOCK:
        {
            mJoystickRightButtons[0] = (bool)mJoystickButton[0];
            mJoystickRightButtons[1] = (bool)mJoystickButton[1];
            mJoystickRightButtons[2] = (bool)mJoystickButton[3];
            mJoystickRightButtons[3] = (bool)mJoystickButton[2];

            mJoystickRearButtons[0] = (bool)mJoystickButton[4];
            mJoystickRearButtons[1] = (bool)mJoystickButton[5];
            mJoystickRearButtons[2] = (bool)mJoystickButton[6];
            mJoystickRearButtons[3] = (bool)mJoystickButton[7];

            mJoystickFunctionButtons[0] = (bool)mJoystickButton[8];
            mJoystickFunctionButtons[1] = (bool)mJoystickButton[9];
            break;
        }
        case XBOX_CONTROLLER:
        {
            mJoystickRightButtons[0] = (bool)mJoystickButton[0];
            mJoystickRightButtons[1] = (bool)mJoystickButton[1];
            mJoystickRightButtons[2] = (bool)mJoystickButton[2];
            mJoystickRightButtons[3] = (bool)mJoystickButton[3];

            mJoystickRearButtons[0] = (bool)mJoystickButton[4];
            mJoystickRearButtons[1] = (bool)mJoystickButton[5];
            if(mJoystickAxis[2] > 0)
            {
                mJoystickRearButtons[2] = true;
            }
            else
            {
                mJoystickRearButtons[2] = false;
            }
            if(mJoystickAxis[5] > 0)
            {
                mJoystickRearButtons[3] = true;
            }
            else
            {
                mJoystickRearButtons[3] = false;
            }

            mJoystickFunctionButtons[0] = (bool)mJoystickButton[6];
            mJoystickFunctionButtons[1] = (bool)mJoystickButton[7];
            break;
        }
    }
}

void Command::printJoystickValue()
{
    std::cout << "axis/10000: ";
    for (size_t i(0); i < mJoystickAxis.size(); ++i)
    {
        std::cout << " " << std::setw(2) << (double)mJoystickAxis[i] / 10000;
    }
    std::cout << "  button: ";
    for (size_t i(0); i < mJoystickButton.size(); ++i)
    {
        std::cout << " " << (double)mJoystickButton[i];
    }
    std::cout << std::endl;
}

void Command::mappingJoystickCommand()
{
    switch(sharedMemory->FSMState)
    {
    case FSM_INITIAL:
    {
        if(mJoystickFunctionButtons[1])
        {
//            std::cout<<"[FSM] : INITIALIZE"<<std::endl;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CAN_ON;
            sharedMemory->FSMState = FSM_READY;
        }
        break;
    }
    case FSM_VISUAL_CAN_READY:
    {
        /// escape motor off
//        std::cout<<"[FSM_VISUAL_CAN_READY]"<<std::endl;
        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_READY:
    {
//        std::cout<<"[FSM_READY]"<<std::endl;
        /// stand up
        if(mJoystickRearButtons[3] && mJoystickLeftButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = HOME;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_ISOLATION:
    {
//        std::cout<<"[FSM_ISOLATION]"<<std::endl;
        /// if end the sit down, go FSM_READY
        if(sharedMemory->bIsEndHome == true && sharedCommand->userCommand == READY)
        {
            sharedMemory->FSMState = FSM_READY;
            sharedMemory->bIsEndHome = false;
        }
        /// if end the stand up, go FSM_CONST_STAND
        if(sharedMemory->bIsEndHome == true && sharedCommand->userCommand == HOME)
        {
            std::cout<<"bISEndHome"<<sharedMemory->bIsEndHome<<std::endl;
            sharedMemory->FSMState = FSM_CONST_STAND;
            sharedMemory->bIsEndHome = false;
        }
        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_CONST_STAND:
    {
//        std::cout<<"[FSM_CONST_STAND]"<<std::endl;
        /// sit down
        if(mJoystickRearButtons[3] && mJoystickLeftButtons[0])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = READY;
            sharedMemory->FSMState = FSM_ISOLATION;
        }
        /// go FSM_TROT_SLOW
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[2])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT_SLOW;
            sharedMemory->FSMState = FSM_TROT_SLOW;
        }
        /// go FSM_TROT_FAST
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[0])
        {
            std::cout<<"go FSM_OVERLAP_TROT_SLOW"<<std::endl;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT_FAST;
            sharedMemory->FSMState = FSM_TROT_FAST;
        }
        /// go DESIRED_GAIT_OVERLAP_TROT_FAST
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_OVERLAP_TROT_FAST;
            sharedMemory->FSMState = FSM_OVERLAP_TROT_FAST;
        }
        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
//        sharedMemory->baseDesiredEulerVelocity[0] = -mJoystickLeftAxis[1] / 50000.0;
//        sharedMemory->baseDesiredEulerVelocity[1] = mJoystickRightAxis[1] / 50000.0;
        sharedMemory->baseDesiredEulerVelocity[2] = -mJoystickRightAxis[0] / 50000.0;

        for (int index = 0 ; index < 3 ; index++)
        {
            if((sharedMemory->baseEulerPosition[index] > PI/10 + mBaseReferenceEulerPosition[index]) && (sharedMemory->baseDesiredEulerVelocity[index] > 0.0))
            {
                sharedMemory->baseDesiredEulerVelocity[index] = 0.0;
            }
            else if ((sharedMemory->baseEulerPosition[index] < -PI/10 + mBaseReferenceEulerPosition[index]) && (sharedMemory->baseDesiredEulerVelocity[index] < 0.0))
            {
                sharedMemory->baseDesiredEulerVelocity[index] = 0.0;
            }
        }
        break;
    }
    case FSM_STAND:
    {
//        std::cout<<"[FSM_STAND]"<<std::endl;
        /// go FSM_CONST_STAND
        if(mJoystickRearButtons[1] && mJoystickLeftButtons[1])
        {
            sharedMemory->FSMState = FSM_CONST_STAND;
            mBaseReferenceEulerPosition[2] = sharedMemory->baseEulerPosition[2];
        }
/*
        /// x, y, yaw axis trot
        if((abs(mLocalVelocity) > 0.04)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.04))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
*/
/*   /// trot change button
        if(mJoystickRearButtons[1] && mJoystickLeftButtons[1])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }*/
        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }


        sharedMemory->baseLocalDesiredVelocity[0] = -mJoystickLeftAxis[1] / 100000.0;
        sharedMemory->baseDesiredVelocity[0] = sharedMemory->baseLocalDesiredVelocity[0] * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = sharedMemory->baseLocalDesiredVelocity[0] * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = -mJoystickRightAxis[0] / 75000.0;
        break;
    }
    case FSM_TROT_SLOW:
    {
//        sharedMemory->baseDesiredEulerPosition[0] = 0.0;
//        sharedMemory->baseDesiredEulerPosition[1] = 0.0;
/*
        /// stand
        if((sqrt(pow(sharedMemory->baseVelocity[0],2)+pow(sharedMemory->baseVelocity[1],2)) < 0.025) && abs(sharedMemory->baseEulerVelocity[2]) < 0.04
           && !((abs(mLocalVelocity) > 0.00004)||(abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.000004)))
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// go FSM_CONST_STAND
        if(mJoystickRearButtons[1] && mJoystickLeftButtons[1])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
            mBaseReferenceEulerPosition[2] = sharedMemory->baseEulerPosition[2];
        }
        /// go FSM_TROT_FAST
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[0])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT_FAST;
            sharedMemory->FSMState = FSM_TROT_FAST;
        }
        /// go FSM_OVERLAP_TROT_FAST
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_OVERLAP_TROT_FAST;
            sharedMemory->FSMState = FSM_OVERLAP_TROT_FAST;
        }
/*
   /// stand change button
        if(mJoystickRearButtons[1] && mJoystickLeftButtons[2])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        if (mJoystickButton[0])
        {
            sharedMemory->isRamp = true;
        }

        if (mJoystickButton[1])
        {
            sharedMemory->isRamp = false;
        }

        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        sharedMemory->baseLocalDesiredVelocity[0] = MAFbaseLocalDesiredVel0.GetFilteredVar(-mJoystickLeftAxis[1] / 50000.0); // 0.65 m/s
        sharedMemory->baseDesiredVelocity[0] = sharedMemory->baseLocalDesiredVelocity[0] * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = sharedMemory->baseLocalDesiredVelocity[0] * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = MAFbaseDesiredEulerVelocity.GetFilteredVar(-mJoystickRightAxis[0] / 50000.0);

        break;
    }
    case FSM_TROT_FAST:
    {
        /// go FSM_CONST_STAND
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[1])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
            mBaseReferenceEulerPosition[2] = sharedMemory->baseEulerPosition[2];
        }
        /// go FSM_TROT_SLOW
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[2])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT_SLOW;
            sharedMemory->FSMState = FSM_TROT_SLOW;
        }
        /// go FSM_OVERLAP_TROT_FAST
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_OVERLAP_TROT_FAST;
            sharedMemory->FSMState = FSM_OVERLAP_TROT_FAST;
        }
        if (mJoystickButton[0])
        {
            sharedMemory->isRamp = true;
        }

        if (mJoystickButton[1])
        {
            sharedMemory->isRamp = false;
        }

        /// motor off
        if (mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        if(-mJoystickLeftAxis[1] > 0)
        {
            // front desired velocity
            sharedMemory->baseLocalDesiredVelocity[0] = MAFbaseLocalDesiredVel0.GetFilteredVar(-mJoystickLeftAxis[1] / 32000.0); // 1 m/s
        }
        else
        {
            sharedMemory->baseLocalDesiredVelocity[0] = MAFbaseLocalDesiredVel0.GetFilteredVar(-mJoystickLeftAxis[1] / 50000.0); // 0.65 m/s
        }
        sharedMemory->baseDesiredVelocity[0] = sharedMemory->baseLocalDesiredVelocity[0] * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = sharedMemory->baseLocalDesiredVelocity[0] * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = MAFbaseDesiredEulerVelocity.GetFilteredVar(-mJoystickRightAxis[0] / 50000.0);
        break;
    }
    case FSM_OVERLAP_TROT_FAST:
    {
        /// go FSM_CONST_STAND
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[1])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_CONST_STAND;
            mBaseReferenceEulerPosition[2] = sharedMemory->baseEulerPosition[2];
        }
        /// go FSM_TROT
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[2])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT_SLOW;
            sharedMemory->FSMState = FSM_TROT_SLOW;
        }
        /// go FSM_TROT_FAST
        if (mJoystickRearButtons[1] && mJoystickLeftButtons[0])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT_FAST;
            sharedMemory->FSMState = FSM_TROT_FAST;
        }
        if (mJoystickButton[0])
        {
            sharedMemory->isRamp = true;
        }

        if (mJoystickButton[1])
        {
            sharedMemory->isRamp = false;
        }

        /// motor off
        if (mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = SIMUL_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        sharedMemory->baseLocalDesiredVelocity[0] = MAFbaseLocalDesiredVel0.GetFilteredVar(-mJoystickLeftAxis[1] / 75000.0);
        sharedMemory->baseDesiredVelocity[0] = sharedMemory->baseLocalDesiredVelocity[0] * cos(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredVelocity[1] = sharedMemory->baseLocalDesiredVelocity[0] * sin(sharedMemory->baseEulerPosition[2]);
        sharedMemory->baseDesiredEulerVelocity[2] = MAFbaseDesiredEulerVelocity.GetFilteredVar(-mJoystickRightAxis[0] / 50000.0);
        break;
    }
    default:
    {
        std::cout<<"[FSM_default]"<<std::endl;
        /// motor off
        if(mJoystickRearButtons[2] && mJoystickRearButtons[3])
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = FSM_VISUAL_CAN_READY;
        }
        break;
    }

    }
}