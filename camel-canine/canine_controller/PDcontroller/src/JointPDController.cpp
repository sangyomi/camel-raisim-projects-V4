//
// Created by camel on 22. 9. 21.
//

#include <PDcontroller/JointPDController.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

JointPDController::JointPDController()
    : mRefTime(0.0)
    , mHomeState(HOME_NO_ACT)
    , mBackFlipState(BACK_FLIP_NO_ACT)
    , mbGenerateBackFlipTrajectory(true)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = 140.0;
        Kd[motorIdx] = 4.5;
        mTorqueLimit[motorIdx] = TORQUE_LIMIT;
    }
}

void JointPDController::SetPDgain(const double& kp, const double& kd)
{
    for (int motorIdx = 0; motorIdx < MOTOR_NUM; motorIdx++)
    {
        Kp[motorIdx] = kp;
        Kd[motorIdx] = kd;
    }
}

void JointPDController::DoHomeControl()
{
    updateState();

    updateHomeTrajectory();

    setHomeTrajectory();

    computeControlInput();

    SetControlInput();
}

void JointPDController::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
        mBaseEulerPosition[idx] = sharedMemory->baseEulerPosition[idx];
        mBaseEulerVelocity[idx] = sharedMemory->baseEulerVelocity[idx];
    }

    for (int leg=0; leg<4; leg++)
    {
        for (int mt=0; mt<3; mt++)
        {
            mMotorPosition[leg][mt] = sharedMemory->motorPosition[leg*3+mt];
            mMotorVelocity[leg][mt] = sharedMemory->motorVelocity[leg*3+mt];
        }
    }

    mInitState << mBaseEulerPosition[0], mBaseEulerPosition[1], mBaseEulerPosition[2],
        mBasePosition[0], mBasePosition[1], mBasePosition[2],
        mBaseEulerVelocity[0], mBaseEulerVelocity[1], mBaseEulerVelocity[2],
        mBaseVelocity[0], mBaseVelocity[1], mBaseVelocity[2], -9.81;

    mDesiredState.setZero();
    mDesiredState[5] = 0.3;
}

void JointPDController::InitHomeStandUpTrajectory()
{
    mHomeState = HOME_STAND_UP_PHASE1;
}

void JointPDController::InitHomeStandDownTrajectory()
{
    mHomeState = HOME_STAND_DOWN_PHASE1;
}

void JointPDController::InitBackFlipTrajectory()
{
    mBackFlipState = BACK_FLIP_STAND_DOWN_PHASE;
}

void JointPDController::DoBackFlipControl()
{
    updateState();

    updateBackFlipTrajectory();

    setBackFlipTrajectory();

    computeControlInput();

    SetControlInput();
}


void JointPDController::computeControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] = Kp[index] * (mDesiredPosition[index] - sharedMemory->motorPosition[index])
            + Kd[index] * (mDesiredVelocity[index] - sharedMemory->motorVelocity[index]);
    }
}

void JointPDController::updateHomeTrajectory()
{
    switch(mHomeState)
    {
    case HOME_NO_ACT:
        break;
    case HOME_STAND_UP_PHASE1:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 90;
            double homeKnee = -157;
            double timeDuration = 0.9;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_STAND_UP_PHASE2;
        break;
    case HOME_STAND_UP_PHASE2:
        if(sharedMemory->localTime > mRefTime + 0.1)
        {
            mHomeState = HOME_STAND_UP_PHASE3;
        }
        break;
    case HOME_STAND_UP_PHASE3:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 60;
            double homeKnee = -120.0;
            double timeDuration = 1.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mDesiredPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mDesiredPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mDesiredPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_NO_ACT;
        break;
    case HOME_STAND_DOWN_PHASE1:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 90;
            double homeKnee = -150;
            double timeDuration = 2.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mDesiredPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mDesiredPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mDesiredPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_STAND_DOWN_PHASE2;
        break;
    case HOME_STAND_DOWN_PHASE2:
        if(sharedMemory->localTime > mRefTime + 0.5)
        {
            mHomeState = HOME_STAND_DOWN_PHASE3;
        }
        break;
    case HOME_STAND_DOWN_PHASE3:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 124;
            double homeKnee = -155;
            double timeDuration = 1.5;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mDesiredPosition[idx * 3], pow(-1,idx)*5*D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mDesiredPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mDesiredPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        mHomeState = HOME_NO_ACT;
        break;
    default:
        break;
    }
}

void JointPDController::updateBackFlipTrajectory()
{
    switch(mBackFlipState)
    {
    case BACK_FLIP_NO_ACT:
        mbGenerateBackFlipTrajectory = true;
        break;
    case BACK_FLIP_STAND_DOWN_PHASE:
        if(mbGenerateBackFlipTrajectory)
        {
            for (int idx = 0; idx < 4; idx++)
            {
                double homeHip = 75;
                double homeKnee = -150;
                double timeDuration = 1.0;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            mbGenerateBackFlipTrajectory = false;
        }
        else
        {
            if(sharedMemory->localTime > mRefTime)
            {
                mBackFlipState = BACK_FLIP_STAND_UP_PHASE;
                mbGenerateBackFlipTrajectory = true;
            }
        }
        break;
    case BACK_FLIP_STAND_UP_PHASE:
        if(mbGenerateBackFlipTrajectory)
        {
            double timeDuration = 0.15;
            for (int idx = 0; idx < 2; idx++)
            {
                double homeHip = 20;
                double homeKnee = -20;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            for (int idx = 2; idx < 4; idx++)
            {
                double homeHip = 75;
                double homeKnee = -70;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            mbGenerateBackFlipTrajectory = false;
        }
        else
        {
            if(sharedMemory->localTime > mRefTime)
            {
                mBackFlipState = BACK_FLIP_JUMP_UP_PHASE;
                mbGenerateBackFlipTrajectory = true;
            }
        }
        break;
    case BACK_FLIP_JUMP_UP_PHASE:
        if(mbGenerateBackFlipTrajectory)
        {
            double timeDuration = 0.15;
            for (int idx = 0; idx < 2; idx++)
            {
                double homeHip = 75;
                double homeKnee = -150;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            for (int idx = 2; idx < 4; idx++)
            {
                double homeHip = 110;
                double homeKnee = -20;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            mbGenerateBackFlipTrajectory = false;
        }
        else
        {
            if(sharedMemory->localTime > mRefTime)
            {
                mBackFlipState = BACK_FLIP_BALANCE_PHASE;
                mbGenerateBackFlipTrajectory = true;
            }
        }
        break;
    case BACK_FLIP_BALANCE_PHASE:
        if(mbGenerateBackFlipTrajectory)
        {
            double timeDuration = 0.15;
            for (int idx = 0; idx < 2; idx++)
            {
                double homeHip = 75;
                double homeKnee = -150;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            for (int idx = 2; idx < 4; idx++)
            {
                double homeHip = 75;
                double homeKnee = -150;
                mRefTime = sharedMemory->localTime + timeDuration;
                mCubicTrajectoryGen[idx * 3].updateTrajectory(sharedMemory->motorPosition[idx * 3], 0.0, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 1], homeHip * D2R, sharedMemory->localTime, timeDuration);
                mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(sharedMemory->motorPosition[idx * 3 + 2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
            }
            mbGenerateBackFlipTrajectory = false;
        }
        else
        {
            if(sharedMemory->localTime > mRefTime)
            {
                mBackFlipState = BACK_FLIP_NO_ACT;
                mbGenerateBackFlipTrajectory = true;
            }
        }
        break;
    default:
        break;
    }
}

void JointPDController::setHomeTrajectory()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mDesiredPosition[index] = mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime);
        mDesiredVelocity[index] = mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime);
    }
}

void JointPDController::setBackFlipTrajectory()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mDesiredPosition[index] = mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime);
        mDesiredVelocity[index] = mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime);
    }
}

void JointPDController::SetControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        if (mTorque[index] > mTorqueLimit[index])
        {
            mTorque[index] = mTorqueLimit[index];
        }
        else if (mTorque[index] < -mTorqueLimit[index])
        {
            mTorque[index] = -mTorqueLimit[index];
        }
        sharedMemory->motorDesiredTorque[index] = mTorque[index];
    }
}

const double* JointPDController::GetTorque() const
{
    return mTorque;
}
