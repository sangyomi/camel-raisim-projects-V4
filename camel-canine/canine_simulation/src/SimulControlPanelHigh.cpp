//
// Created by hs on 23. 2. 9.
//

#include <canine_simulation/SimulControlPanelHigh.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulControlPanelHigh::SimulControlPanelHigh()
    : mStand(MPC_HORIZON, 30, Vec4<int>(30,30,30,30), Vec4<int>(30,30,30,30))
    , mTrotSlow(MPC_HORIZON, 30, Vec4<int>(15,0,0,15), Vec4<int>(15,15,15,15))
    , mTrotFast(MPC_HORIZON, 20, Vec4<int>(10,0,0,10), Vec4<int>(10,10,10,10))
    , mOverlapTrotFast(MPC_HORIZON, 40, Vec4<int>(10,30,30,10), Vec4<int>(30,30,30,30))
{
}

void SimulControlPanelHigh::ControllerFunction()
{
    startGaitSelectPanel();
    startHighControlPanel();
}

void SimulControlPanelHigh::startGaitSelectPanel()
{
    switch (sharedMemory->gaitState)
    {
        case STAND:
        {
            if (sharedMemory->gaitChangeFlag)
            {
                startGaitChangePanel(&mStand);
            }
            else
            {
                mStand.GetGaitTable();
            }
            break;
        }
        case TROT_SLOW:
        {
            if (sharedMemory->gaitChangeFlag)
            {
                startGaitChangePanel(&mTrotSlow);
            }
            else
            {
                mTrotSlow.GetGaitTable();
            }
            break;
        }
        case TROT_FAST:
        {
            if (sharedMemory->gaitChangeFlag)
            {
                startGaitChangePanel(&mTrotFast);
            }
            else
            {
                mTrotFast.GetGaitTable();
            }
            break;
        }
        case OVERLAP_TROT_FAST:
        {
            if (sharedMemory->gaitChangeFlag)
            {
                startGaitChangePanel(&mOverlapTrotFast);
            }
            else
            {
                mOverlapTrotFast.GetGaitTable();
            }
            break;
        }
        default:
        {
            break;
        }
    }
    mIteration++;
}

void SimulControlPanelHigh::startGaitChangePanel(OffsetGait* currentGait)
{
    switch(sharedCommand->gaitCommand)
    {
        case DESIRED_GAIT_STAND:
        {
            if (currentGait->GetGaitTableTrans())
            {
                sharedMemory->gaitChangeFlag = false;
                sharedMemory->gaitState = STAND;
                sharedMemory->gaitPeriod = 0.6;
                sharedMemory->swingPeriod = 0.0;
                sharedMemory->standPeriod = 0.6;
            }
            break;
        }
        case DESIRED_GAIT_TROT_SLOW:
        {
            if (currentGait->GetGaitTableTrans())
            {
                sharedMemory->gaitChangeFlag = false;
                sharedMemory->gaitState = TROT_SLOW;
                sharedMemory->gaitPeriod = 0.6;
                sharedMemory->swingPeriod = 0.3;
                sharedMemory->standPeriod = 0.3;
            }
            break;
        }
        case DESIRED_GAIT_TROT_FAST:
        {
            if (currentGait->GetGaitTableTrans())
            {
                sharedMemory->gaitChangeFlag = false;
                sharedMemory->gaitState = TROT_FAST;
                sharedMemory->gaitPeriod = 0.4;
                sharedMemory->swingPeriod = 0.2;
                sharedMemory->standPeriod = 0.2;
            }
            break;
        }
        case DESIRED_GAIT_OVERLAP_TROT_FAST:
        {
            if (currentGait->GetGaitTableTrans())
            {
                sharedMemory->gaitChangeFlag = false;
                sharedMemory->gaitState = OVERLAP_TROT_FAST;
                sharedMemory->gaitPeriod = 0.8;
                sharedMemory->swingPeriod = 0.2;
                sharedMemory->standPeriod = 0.6;
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void SimulControlPanelHigh::startHighControlPanel()
{
    switch (sharedMemory->HighControlState)
    {
        case STATE_HIGH_CONTROL_STOP:
        {
            break;
        }
        case STATE_DEFAULT_CONTROL:
        {
            MPC.DoControl();
            break;
        }
        case STATE_HIGH_HOME_STAND_UP_READY:
        {
            MPC.InitUpTrajectory();

            sharedMemory->HighControlState = STATE_HIGH_DO_CONTROL;
            break;
        }
        case STATE_HIGH_HOME_STAND_DOWN_READY:
        {
            MPC.InitDownTrajectory();
            sharedMemory->HighControlState = STATE_HIGH_DO_CONTROL;
            break;
        }
        case STATE_HIGH_DO_CONTROL:
        {
            MPC.DoControl();
            sharedMemory->LowControlState = STATE_LOW_HOME_CONTROL;
            break;
        }
        default:
            break;
    }
}