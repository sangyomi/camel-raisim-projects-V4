//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/LowControlMain.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

LowControlMain::LowControlMain()
    : mIteration(0)
    , mAlpha(0)
    , mRefTime(0)
    , bStandUp(false)
    , bStandDown(false)
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mCalTorque[MOTOR_NUM] = 0;
    }
}

void LowControlMain::ControllerFunction()
{
    sharedMemory->localTime = mIteration * LOW_CONTROL_dT;
    mIteration++;

    if (mAlpha > 1)
    {
        std::cout<<"[LOW CONTROL] Home up is ended."<<std::endl;
        sharedMemory->HighControlState = STATE_DEFAULT_CONTROL;
        sharedMemory->LowControlState = STATE_LOW_CONTROL_START;
        sharedMemory->bIsEndHome = true;
        bStandDown = false;
        bStandUp = false;
        mAlpha = 1;
    }
    else if (mAlpha < 0)
    {
        std::cout<<"[LOW CONTROL] Home down is ended."<<std::endl;
        sharedMemory->bIsEndHome = true;
        bStandDown = false;
        bStandUp = false;
        mAlpha = 0;
    }
    else
    {
        if (bStandUp)
        {
            mAlpha = (STAND_UP_TIME - mRefTime + sharedMemory->localTime) / STAND_UP_TIME;
        }
        if (bStandDown)
        {
            mAlpha = 1 - ((STAND_DOWN_TIME - mRefTime + sharedMemory->localTime) / STAND_DOWN_TIME);
        }
    }

    switch (sharedMemory->LowControlState)
    {
    case STATE_LOW_CONTROL_STOP:
    {
        break;
    }
    case STATE_LOW_CONTROL_START:
    {
        LowController.DoControl();
        break;
    }
    case STATE_LOW_HOME_STAND_UP_READY:
    {
        PDcontrol.InitHomeStandUpTrajectory();
        mRefTime = sharedMemory->localTime + STAND_UP_TIME;
        bStandUp = true;
        break;
    }
    case STATE_LOW_HOME_STAND_DOWN_READY:
    {
        PDcontrol.InitHomeStandDownTrajectory();
        mRefTime = sharedMemory->localTime + STAND_DOWN_TIME;
        bStandDown = true;
    }
    case STATE_LOW_HOME_CONTROL:
    {
        LowController.DoControl();
        PDcontrol.DoHomeControl();
        for (int index = 0; index < MOTOR_NUM; index++)
        {
            mCalTorque[index] = mAlpha * LowController.GetTorque()[index] + (1 - mAlpha) * PDcontrol.GetTorque()[index];
            if (mCalTorque[index] > TORQUE_LIMIT)
            {
                mCalTorque[index] = TORQUE_LIMIT;
            }
            else if (mCalTorque[index] < -TORQUE_LIMIT)
            {
                mCalTorque[index] = -TORQUE_LIMIT;
            }
            sharedMemory->motorDesiredTorque[index] = mCalTorque[index];
        }
        break;
    }
    default:
        break;
    }
}