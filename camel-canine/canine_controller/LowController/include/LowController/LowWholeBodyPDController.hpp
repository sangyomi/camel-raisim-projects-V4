//
// Created by jaehoon on 23. 4. 14.
//

#ifndef CAMEL_RAISIM_PROJECTS_LOWWHOLEBODYPDCONTROLLER_HPP
#define CAMEL_RAISIM_PROJECTS_LOWWHOLEBODYPDCONTROLLER_HPP

#include <string.h>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotMath.hpp>

#include <ControlUtils/CubicSwingLeg.hpp>
#include <ControlUtils/SwingLeg.hpp>

class LowWholeBodyPDController{
public:
    LowWholeBodyPDController();

    void DoControl();
    double* GetTorque();

private:
    void updateState();
    void setControlInput();
    void setLegControl();
    void getJointPos(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg, const bool stand);
    void getJointVel(Vec3<double>& resultVel, Vec3<double> desiredPos, const int& leg, const bool stand);

private:
    u_int64_t mIteration;
    int mGaitTable[4];
    const int mTorqueLimit;
    bool bIsFirstRunSwing[4];
    bool bIsFirstRunStand[4];
    bool bIsFirstHome[4];
//    CubicSwingLeg SwingLegTrajectory;
    SwingLeg SwingLegTrajectory;
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

    Vec3<double> mBasePosition;
    Vec3<double> mBaseDesiredPosition;
    Vec3<double> mBaseVelocity;
    Vec3<double> mBaseDesiredVelocity;
    Vec3<double> mBaseDesiredEulerVelocity;
    Vec3<double> mBaseEulerVelocity;
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mBodyFootPosition[4];
    Vec3<double> mGlobalFootPosition[4];
    Vec3<double> mDesiredFootPosition[4];
    Vec3<double> mSwingFootDesiredVelocity[4];
    Vec3<double> mPrevDesiredFootPosition[4];
    Vec3<double> mSwingFootPosition[4];
    Vec4<double> mBaseQuaternion;
    Vec4<double> mBaseDesiredQuaternion;
    Vec3<double> mShoulderPosition[4];
    Vec3<double> mHipPosition[4];

    Vec13<double> mInitState;
    Vec13<double> mDesiredState;

    Mat3<double> Body2GlobalTransMat;
    double mReturnTorque[MOTOR_NUM];
};


#endif //CAMEL_RAISIM_PROJECTS_LOWWHOLEBODYPDCONTROLLER_HPP
