//
// Created by jaehoon on 23. 5. 21.
//

#include <ControlUtils/CubicSwingLegLocal.hpp>

extern pSHM sharedMemory;

CubicSwingLegLocal::CubicSwingLegLocal()
    : mbIsNewTrajectory{false, false, false, false}
    , mLiftUpHeight{-0.22, -0.22, -0.22, -0.22 }
    , mTargetHeight{-0.34, -0.34, -0.34, -0.34}
{
}

void CubicSwingLegLocal::SetParameters()
{
    mTimeDuration = sharedMemory->swingPeriod;
    mLiftUpDuration = mTimeDuration/2;
}

void CubicSwingLegLocal::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    mReferenceTime[leg] = sharedMemory->localTime;
    mCubicFootTrajectoryGenerator[leg*3].updateTrajectory(initPos[0],desiredPos[0],sharedMemory->localTime,mTimeDuration);
    mCubicFootTrajectoryGenerator[leg*3+1].updateTrajectory(initPos[1],desiredPos[1],sharedMemory->localTime,mTimeDuration);
    mCubicFootTrajectoryGenerator[leg*3+2].updateTrajectory(initPos[2] - POS_FOO_GND_Z,mLiftUpHeight[leg],sharedMemory->localTime,mLiftUpDuration);
    mbIsNewTrajectory[leg] = true;
}

void CubicSwingLegLocal::GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg)
{
    desiredPosition[0] = mCubicFootTrajectoryGenerator[leg*3].getPositionTrajectory(sharedMemory->localTime);
    desiredPosition[1] = mCubicFootTrajectoryGenerator[leg*3+1].getPositionTrajectory(sharedMemory->localTime);
    desiredPosition[2] = mCubicFootTrajectoryGenerator[leg*3+2].getPositionTrajectory(sharedMemory->localTime);
    if((sharedMemory->localTime > mLiftUpDuration + mReferenceTime[leg]) && mbIsNewTrajectory[leg])
    {
        mbIsNewTrajectory[leg] = false;
        mCubicFootTrajectoryGenerator[leg*3+2].updateTrajectory(desiredPosition[2],mTargetHeight[leg],sharedMemory->localTime,mTimeDuration - mLiftUpDuration);
    }
}

void CubicSwingLegLocal::GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg)
{
    desiredVelocity[0] = mCubicFootTrajectoryGenerator[leg*3].getVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[1] = mCubicFootTrajectoryGenerator[leg*3+1].getVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[2] = mCubicFootTrajectoryGenerator[leg*3+2].getVelocityTrajectory(sharedMemory->localTime);
}

void CubicSwingLegLocal::ReplanningXY(Vec3<double>& desiredPosition, const int& leg)
{
    mCubicFootTrajectoryGenerator[leg*3].Replanning(desiredPosition[0], sharedMemory->localTime);
    mCubicFootTrajectoryGenerator[leg*3+1].Replanning(desiredPosition[1], sharedMemory->localTime);
}