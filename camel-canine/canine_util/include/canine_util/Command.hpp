#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "Filter.hpp"
#include "JoystickInfo.hpp"

class Command {
public:
    Command();

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
    double mLocalVelocity;
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


#endif //RAISIM_COMMAND_H
