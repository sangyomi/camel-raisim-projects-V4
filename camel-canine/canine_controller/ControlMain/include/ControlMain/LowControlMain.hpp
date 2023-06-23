//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_LOWCONTROLMAIN_HPP
#define RAISIM_LOWCONTROLMAIN_HPP

#include <stdint.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>

#include <LowController/LowPDcontrol.hpp>
#include <PDcontroller/JointPDController.hpp>
#include <ControlUtils/Gait.hpp>

class LowControlMain{
public:
    LowControlMain();

    void ControllerFunction();

private:
    uint64_t mIteration;
    double mAlpha;
    double mCalTorque[MOTOR_NUM];
    double mRefTime;
    bool bStandUp;
    bool bStandDown;

    LowPDcontrol LowController;
    JointPDController PDcontrol;
};

#endif //RAISIM_CONTROLSTATE_H
