//
// Created by hs on 22. 8. 10.
//

#include <ControlUtils/Gait.hpp>

extern pSHM sharedMemory;

OffsetGait::OffsetGait(int mpcHorizon, uint64_t gaitPeriod, Vec4<int> offsets, Vec4<int> durations)
    : mOffsets(offsets.array())
    , mDurations(durations.array())
    , mHorizon(mpcHorizon)
    , mGaitPeriod(gaitPeriod)
    , mIteration(0)
{
}

OffsetGait::~OffsetGait() noexcept
{
}

void OffsetGait::GetGaitTable()
{
    for(int i = 0; i < mHorizon; i++)
    {
        int iter = (i + mIteration) % mGaitPeriod;
        Eigen::Array4i progress = iter - mOffsets;
        for(int j = 0; j < 4; j++)
        {
            if(progress[j] < 0)
            {
                progress[j] += mGaitPeriod;
            }
            if(progress[j] < mDurations[j])
            {
                mGaitTable[i*4 + j] = 1;
            }
            else
            {
                mGaitTable[i*4 + j] = 0;
            }
        }
    }
    for(int i = 0 ; i< MPC_HORIZON * 4 ; i++)
    {
        sharedMemory->gaitTable[i] = mGaitTable[i];
    }

    mIteration = (++mIteration)%mGaitPeriod;

    for(int leg=0; leg<4; leg++)
    {
        sharedMemory->solvedGRF[leg][2] =7*mGaitTable[leg];
    }
}

bool OffsetGait::GetGaitTableTrans()
{
    if (mGaitPeriod-mIteration <= mHorizon)
    {
        for(int i=1; i<5; i++)
        {
            sharedMemory->gaitTable[(i-1)*4] = sharedMemory->gaitTable[i*4];
            sharedMemory->gaitTable[(i-1)*4+1] = sharedMemory->gaitTable[i*4+1];
            sharedMemory->gaitTable[(i-1)*4+2] = sharedMemory->gaitTable[i*4+2];
            sharedMemory->gaitTable[(i-1)*4+3] = sharedMemory->gaitTable[i*4+3];
        }
        for(int i=16; i<20; i++)
        {
            sharedMemory->gaitTable[i] = 1;
        }
        ++mIteration;
    }
    else
    {
        GetGaitTable();
    }

    if (mGaitPeriod == mIteration)
    {
        mIteration = 0;
        return true;
    }
    else
    {
        return false;
    }
}