//
// Created by hs on 22. 8. 10.
//

#ifndef RAISIM_GAIT_H
#define RAISIM_GAIT_H

#include <iostream>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/SharedMemory.hpp>

class OffsetGait{
public:
    OffsetGait(int mpcHorizon, uint64_t cyclePeriod, Vec4<int> offsets, Vec4<int> durations);
    ~OffsetGait();
    void GetGaitTable();
    bool GetGaitTableTrans();

private:
    int mGaitTable[MPC_HORIZON*4];
    int mHorizon;
    int mGaitPeriod;
    int mIteration;
    Eigen::Array4i mOffsets;
    Eigen::Array4i mDurations;
};



#endif //RAISIM_GAIT_H
