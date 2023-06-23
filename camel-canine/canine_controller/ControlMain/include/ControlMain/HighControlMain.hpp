//
// Created by hs on 23. 2. 9.
//

#ifndef RAISIM_HIGHCONTROLMAIN_HPP
#define RAISIM_HIGHCONTROLMAIN_HPP

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <convexMPC/MPCController.hpp>
#include <ControlUtils/Gait.hpp>

class HighControlMain
{
public:
    HighControlMain();

    void ControllerFunction();

private:
    void startGaitSelectPanel();
    void startGaitChangePanel(OffsetGait* currentGait);
    void startHighControlPanel();

private:
    uint64_t mIteration;

    MPCController MPC;
    OffsetGait mStand, mTrotSlow, mTrotFast, mOverlapTrotFast;
};

#endif //RAISIM_HIGHCONTROLMAIN_HPP
