//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULCOMMAND_HPP
#define RAISIM_SIMULCOMMAND_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/JoystickInfo.hpp>

class SimulCommand
{
public:
    SimulCommand();
    void commandFunction();

private:
    void initializeJoystick();
    void initializeJoystickOnex();
    void readJoystick();
    void readJoystickOnex();
    void printJoystickValue();
    void mappingJoystickCommand();

private:
    int mJoystickType;
    int mJoystickFd;
    int mJoystickNumOfAxis;
    int mJoystickNumOfButton;
    int mDeadBand;
    char mJoystickName[80];
    std::vector<char> mJoystickButton;
    std::vector<int> mJoystickAxis;

    bool mJoystickRightButtons[4]; // (A, B, X, Y) (X, O, nemo, semo)
    bool mJoystickLeftButtons[4]; // down, right, left, up
    bool mJoystickRearButtons[4]; // L1, R1, L2, R2
    bool mJoystickFunctionButtons[2]; // option, start
    double mJoystickLeftAxis[2]; // left-right, down-up
    double mJoystickRightAxis[2]; // left-right, down-up
    double mBaseReferenceEulerPosition[3];
    enum meJoystick
    {
        DUAL_SHOCK,
        XBOX_CONTROLLER
    };
};

#endif //RAISIM_SIMULCOMMAND_HPP
