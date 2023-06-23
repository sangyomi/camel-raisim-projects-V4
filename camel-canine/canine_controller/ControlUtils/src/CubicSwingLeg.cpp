//
// Created by jaehoon on 23. 4. 14.
//

#include <ControlUtils/CubicSwingLeg.hpp>

extern pSHM sharedMemory;

CubicSwingLeg::CubicSwingLeg()
    : mbIsNewTrajectory{false, false, false, false}
    , mLiftUpHeight{0.1, 0.1, 0.1, 0.1 }
    , mTargetHeight{0.0, 0.0, 0.0, 0.0}
{
}

void CubicSwingLeg::SetParameters()
{
    mTimeDuration = sharedMemory->swingPeriod;
    mLiftUpDuration = mTimeDuration*3/4;
}

void CubicSwingLeg::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    mReferenceTime[leg] = sharedMemory->localTime;
    mCubicFootTrajectoryGenerator[leg*3].updateTrajectory(initPos[0],desiredPos[0],sharedMemory->localTime,mTimeDuration);
    mCubicFootTrajectoryGenerator[leg*3+1].updateTrajectory(initPos[1],desiredPos[1],sharedMemory->localTime,mTimeDuration);
    mCubicFootTrajectoryGenerator[leg*3+2].updateTrajectory(initPos[2],initPos[2] + mLiftUpHeight[leg],sharedMemory->localTime,mLiftUpDuration);
    mbIsNewTrajectory[leg] = true;
    mTargetHeight[leg] = initPos[2];
}

void CubicSwingLeg::GetTrajectory(Vec3<double>& desiredPosition, const Vec3<double>& initPos, const int& leg)
{
    desiredPosition[0] = mCubicFootTrajectoryGenerator[leg*3].getPositionTrajectory(sharedMemory->localTime);
    desiredPosition[1] = mCubicFootTrajectoryGenerator[leg*3+1].getPositionTrajectory(sharedMemory->localTime);
    desiredPosition[2] = mCubicFootTrajectoryGenerator[leg*3+2].getPositionTrajectory(sharedMemory->localTime);
    if((sharedMemory->localTime > mLiftUpDuration + mReferenceTime[leg]) && mbIsNewTrajectory[leg])
    {
        mbIsNewTrajectory[leg] = false;
        mCubicFootTrajectoryGenerator[leg*3+2].updateTrajectory(mLiftUpHeight[leg],mTargetHeight[leg],sharedMemory->localTime,mTimeDuration - mLiftUpDuration);
    }
}

void CubicSwingLeg::GetVelocityTrajectory(double localTime, Vec3<double>& desiredVelocity, const int& leg)
{
    desiredVelocity[0] = mCubicFootTrajectoryGenerator[leg*3].getVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[1] = mCubicFootTrajectoryGenerator[leg*3+1].getVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[2] = mCubicFootTrajectoryGenerator[leg*3+2].getVelocityTrajectory(sharedMemory->localTime);
}
