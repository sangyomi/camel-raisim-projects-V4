//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_LOWPDCONTROL_HPP
#define RAISIM_LOWPDCONTROL_HPP

#include <string.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotMath.hpp>

#include <ControlUtils/SwingLegLocal.hpp>
#include <ControlUtils/CubicSwingLegLocal.hpp>

class LowPDcontrol
{
public:
    LowPDcontrol();

    void DoControl();
    double* GetTorque();

private:
    void updateState();
    void setControlInput();
    void setLegControl();
    void getJointPos(Vec3<double>& resultPos, Vec3<double> desiredPos, const int& leg, bool stand);
    void getJointVel(Vec3<double>& jointVel, const int& leg, bool stand);

private:
    const int mTorqueLimit;
    int mCount[4];
    int mGaitTable[4];

    bool bIsRunTrot[4];
    bool bIsFirstRunStand[4];
    bool bIsFirstHome[4];
//    SwingLegLocal SwingLegTrajectory;       // Bezier Swing Leg Trajectory
    CubicSwingLegLocal SwingLegTrajectory;  // Cubic Swing Leg Trajectory
    Vec3<double> mLegTorque[4];
    Vec3<double> mSwingJointPos;
    Vec3<double> mSwingJointVel;
    Vec3<double> mStandJointPos;
    Vec3<double> mStandJointVel;
    Vec3<double> mSwingPgain;
    Vec3<double> mSwingDgain;
    Vec3<double> mStandPgain;
    Vec3<double> mStandDgain;

    Vec3<double> mTorque[4];
    Vec4<double> mBaseDesiredQuaternion;
    Vec3<double> mBasePosition;
    Vec3<double> mBaseVelocity;
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mBodyFootPosition[4];
    Vec3<double> mGlobalFootPosition[4];
    Vec3<double> mDesiredFootPosition[4];
    Vec3<double> mSwingFootPosition[4];
    Vec3<double> mSwingFootDesiredVelocity[4];
    Vec3<double> mStandFootPosition[4];
    Vec3<double> mHipFootPosition[4];
    Vec4<double> mBaseQuaternion;
    Vec3<double> mHipPosition[4];
    Vec3<double> mShoulderPosition[4];
    Vec3<double> mHip2ShoulderPosition[4];
    Vec3<double> mLocalBaseDesiredEuler[4];
    Vec3<double> mLocalBaseDesiredLinear[4];
    Vec13<double> mInitState;
    Vec13<double> mDesiredState;

    Mat3<double> Body2GlobalTransMat;
    double mReturnTorque[MOTOR_NUM];
    Vec3<double> mLocomotionDesiredVelocity[4];
};

#endif //RAISIM_LOWPDCONTROL_HPP
