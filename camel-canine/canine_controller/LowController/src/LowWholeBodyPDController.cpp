//
// Created by jaehoon on 23. 4. 14.
//

/***
 *  Low Global PD controller
 */
#include <LowController/LowWholeBodyPDController.hpp>
extern pSHM sharedMemory;
LowWholeBodyPDController::LowWholeBodyPDController()
    : mTorqueLimit(30)
    , mIteration(0)
    , bIsFirstRunStand{ true, true, true, true }
    , bIsFirstHome{ true, true, true, true }
    , mSwingPgain{ 60, 60, 60 }
    , mSwingDgain{ 2.0, 2.0, 2.0 }
    , mStandPgain{ 20, 20, 20 }
    , mStandDgain{ 5.0, 5.0, 5.0 }
{
    mTorque->setZero();
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mReturnTorque[idx] = 0;
    }
    mShoulderPosition[LF_IDX] << SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[RF_IDX] << SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[LB_IDX] << -SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[RB_IDX] << -SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mHipPosition[LF_IDX] << HIP_X_POS, HIP_Y_POS, 0;
    mHipPosition[RF_IDX] << HIP_X_POS, -HIP_Y_POS, 0;
    mHipPosition[LB_IDX] << -HIP_X_POS, HIP_Y_POS, 0;
    mHipPosition[RB_IDX] << -HIP_X_POS, -HIP_Y_POS, 0;
    mBaseDesiredQuaternion << 1.0, 0.0, 0.0, 0.0;
    mStandJointVel.setZero();
    mSwingJointVel.setZero();
}
void LowWholeBodyPDController::DoControl()
{
    updateState();
    setLegControl();
    setControlInput();
}
void LowWholeBodyPDController::updateState()
{
    mIteration++;
    for (int leg = 0; leg < 4; leg++)
    {
        mGaitTable[leg] = sharedMemory->gaitTable[leg];
    }
    for (int idx = 0; idx < 3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
        mBaseDesiredPosition[idx] = sharedMemory->baseDesiredPosition[idx];
        mBaseDesiredVelocity[idx] = sharedMemory->baseDesiredVelocity[idx];
        mBaseEulerVelocity[idx] = sharedMemory->baseEulerVelocity[idx];
        mBaseDesiredEulerVelocity[idx] = sharedMemory->baseDesiredEulerVelocity[idx];
    }
    for (int idx = 0; idx < 4; idx++)
    {
        mBaseQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
        mBaseDesiredQuaternion[idx] = sharedMemory->baseDesiredQuartPosition[idx];
        mBodyFootPosition[idx] = sharedMemory->bodyFootPosition[idx];
        mGlobalFootPosition[idx] = sharedMemory->globalFootPosition[idx];
        mGlobalFootPosition[idx][2] += 0.025;
        for (int mt = 0; mt < 3; mt++)
        {
            mMotorPosition[idx][mt] = sharedMemory->motorPosition[idx * 3 + mt];
            mMotorVelocity[idx][mt] = sharedMemory->motorVelocity[idx * 3 + mt];
        }
    }
}
void LowWholeBodyPDController::setLegControl()
{
    for (int leg = 0; leg < 4; leg++)
    {
        if (mGaitTable[leg] == 0)
        {
            if (bIsFirstRunSwing[leg] == true)
            {
                Vec3<double> preDesiredFootPosition = mDesiredFootPosition[leg];
                double H_gain = 1;
                Mat3<double> yawRateRot;
                double yawRateWeight = 1.0;
                double yawRate = sharedMemory->gaitPeriod * 3/4*H_gain* mBaseEulerVelocity[2];
                yawRateRot << cos(yawRateWeight * yawRate), -sin(yawRateWeight * yawRate), 0
                ,sin(yawRateWeight * yawRate), cos(yawRateWeight * yawRate), 0
                ,0, 0, 1;
                double RaibertV = 0.02;
                double RaibertP = 0;

//                mDesiredFootPosition[leg] = yawRateRot * (GetBaseRotationMat(mBaseDesiredQuaternion) * (mShoulderPosition[leg])) + mBasePosition + sharedMemory->gaitPeriod/ 2 * mBaseVelocity;
//                mDesiredFootPosition[leg] = GetBaseRotationMat(mBaseDesiredQuaternion) * (mShoulderPosition[leg]) + mBasePosition;
                Vec3<double> S = RaibertP*(mBasePosition - mBaseDesiredPosition)+RaibertV*(mBaseVelocity - mBaseDesiredVelocity);
                Vec3<double> V = mGaitTable[LF_IDX]*mGlobalFootPosition[LF_IDX] + mGaitTable[RF_IDX]*mGlobalFootPosition[RF_IDX] - mGaitTable[LB_IDX]*mGlobalFootPosition[LB_IDX] - mGaitTable[RB_IDX]*mGlobalFootPosition[RB_IDX];
                mDesiredFootPosition[leg] = yawRateRot * GetBaseRotationMat(mBaseDesiredQuaternion) * (mShoulderPosition[leg]) + mBasePosition + sharedMemory->gaitPeriod*3/4*H_gain * mBaseVelocity
                    + (S - (S.dot(V))/(V.dot(V))*V);
//                mDesiredFootPosition[leg] = yawRateRot * (GetBaseRotationMat(mBaseDesiredQuaternion) * (mShoulderPosition[leg])) + mBasePosition + sharedMemory->gaitPeriod * 1/2 / 2.0 * mBaseVelocity + 0.5 * mBaseVelocity * mBaseEulerVelocity[2];
//                mDesiredFootPosition[leg] = yawRateRot * (GetBaseRotationMat(mBaseDesiredQuaternion) * (mShoulderPosition[leg])) + mBasePosition + sharedMemory->gaitPeriod/ 2 * mBaseVelocity;
                mDesiredFootPosition[leg][2] = 0.025; //desired foot height
                SwingLegTrajectory.SetFootTrajectory(mGlobalFootPosition[leg], mDesiredFootPosition[leg], leg);
                bIsFirstRunSwing[leg] = false;
                bIsFirstRunStand[leg] = true;
            }
            SwingLegTrajectory.GetTrajectory(mSwingFootPosition[leg], mGlobalFootPosition[leg], leg); //global
            getJointPos(mSwingJointPos, mSwingFootPosition[leg], leg, false);
            SwingLegTrajectory.GetVelocityTrajectory(sharedMemory->localTime, mSwingFootDesiredVelocity[leg], leg);
            getJointVel(mSwingJointVel, mSwingJointPos, leg, false);
            for (int mt = 0; mt < 3; mt++)
            {
                sharedMemory->pdTorque[leg][mt] = mSwingPgain[mt] * (mSwingJointPos[mt] - mMotorPosition[leg][mt])
                    + mSwingDgain[mt] * (mSwingJointVel[mt] - mMotorVelocity[leg][mt]);
            }
        }
        else
        {
            if (bIsFirstHome[leg] == true)
            {
                mPrevDesiredFootPosition[leg] = mGlobalFootPosition[leg];
                mDesiredFootPosition[leg] = mGlobalFootPosition[leg];
                bIsFirstRunStand[leg] = false;
                bIsFirstRunSwing[leg] = true;
            }
            else if (bIsFirstRunStand[leg] == true)
            {
                bIsFirstRunSwing[leg] = true;
                mDesiredFootPosition[leg] = mSwingFootPosition[leg];
                mDesiredFootPosition[leg][2] = 0.025;
                bIsFirstRunStand[leg] = false;
            }
            getJointPos(mStandJointPos, mDesiredFootPosition[leg], leg, true);
            getJointVel(mStandJointVel, mStandJointPos, leg, true);
            for (int mt = 0; mt < 3; mt++)
            {
                sharedMemory->pdTorque[leg][mt] = mStandPgain[mt] * (mStandJointPos[mt] - mMotorPosition[leg][mt])
                    + mStandDgain[mt] * (mStandJointVel[mt] - mMotorVelocity[leg][mt]);
            }
        }
    }
    for (int idx = 0; idx < 4; idx++)
    {
        if(!sharedMemory->isNan)
        {
            mTorque[idx][0] = sharedMemory->pdTorque[idx][0] + sharedMemory->mpcTorque[idx][0];
            mTorque[idx][1] = sharedMemory->pdTorque[idx][1] + sharedMemory->mpcTorque[idx][1];
            mTorque[idx][2] = sharedMemory->pdTorque[idx][2] + sharedMemory->mpcTorque[idx][2];
//        mTorque[idx][0] = sharedMemory->pdTorque[idx][0];
//        mTorque[idx][1] = sharedMemory->pdTorque[idx][1];
//        mTorque[idx][2] = sharedMemory->pdTorque[idx][2];
//        mTorque[idx][0] = sharedMemory->mpcTorque[idx][0];
//        mTorque[idx][1] = sharedMemory->mpcTorque[idx][1];
//        mTorque[idx][2] = sharedMemory->mpcTorque[idx][2];
        }
        else
        {
            mTorque[idx].setZero();
        }
    }
}
void LowWholeBodyPDController::getJointPos(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg, const bool stand)
{
    if (bIsFirstHome[leg])
    {
        bIsFirstHome[leg] = false;
    }
    if (stand)
    {
        footPos = GetBaseRotationMatInverse(mBaseDesiredQuaternion) * (footPos - mBaseDesiredPosition) - mHipPosition[leg];
    }
    else
    {
        footPos = GetBaseRotationMatInverse(mBaseQuaternion) * (footPos - mBasePosition) - mHipPosition[leg];
    }
    GetLegInvKinematics(jointPos, footPos, leg);
    sharedMemory->motorDesiredPosition[leg * 3] = jointPos[0];
    sharedMemory->motorDesiredPosition[leg * 3 + 1] = jointPos[1];
    sharedMemory->motorDesiredPosition[leg * 3 + 2] = jointPos[2];
}
void LowWholeBodyPDController::setControlInput()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            if (mTorque[leg][motor] > mTorqueLimit)
            {
                mTorque[leg][motor] = mTorqueLimit;
            }
            else if (mTorque[leg][motor] < -mTorqueLimit)
            {
                mTorque[leg][motor] = -mTorqueLimit;
            }
            sharedMemory->motorDesiredTorque[leg * 3 + motor] = mTorque[leg][motor];
        }
    }
}
void LowWholeBodyPDController::getJointVel(Vec3<double>& resultVel, Vec3<double> desiredPos, const int& leg, const bool stand)
{
    Eigen::Matrix<double,3,3> jacobian;
    Eigen::Matrix<double,3,3> jacobianInverse;
    jacobian.setZero();
    jacobianInverse.setZero();
    Mat3<double> rot = GetBaseRotationMatInverse(mBaseDesiredQuaternion);
    Vec3<double> base2footPosition;
    GetJacobian2(jacobian, desiredPos, leg);
    jacobianInverse = jacobian.inverse();
    if (stand)
    {
        base2footPosition = GetBaseRotationMatInverse(mBaseDesiredQuaternion)*(mDesiredFootPosition[leg] - mBaseDesiredPosition);
        resultVel = jacobianInverse*GetBaseRotationMatInverse(mBaseDesiredQuaternion)*(mBaseDesiredVelocity) + jacobianInverse*mBaseDesiredEulerVelocity.cross(base2footPosition);
    }
    else
    {
        base2footPosition = GetBaseRotationMatInverse(mBaseQuaternion)*(mSwingFootPosition[leg] - mBasePosition);
        resultVel = jacobianInverse*GetBaseRotationMatInverse(mBaseQuaternion)*(mBaseVelocity - mSwingFootDesiredVelocity[leg]) + jacobianInverse*mBaseEulerVelocity.cross(base2footPosition);
    }
    for (int i = 0; i < 3; i++)
    {
        sharedMemory->motorPrevDesiredPosition[leg * 3 + i] = sharedMemory->motorDesiredPosition[leg * 3 + i];
    }
    sharedMemory->motorDesiredVelocity[leg * 3] = resultVel[0];
    sharedMemory->motorDesiredVelocity[leg * 3 + 1] = resultVel[1];
    sharedMemory->motorDesiredVelocity[leg * 3 + 2] = resultVel[2];
}
double* LowWholeBodyPDController::GetTorque()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            mReturnTorque[leg * 3 + motor] = mTorque[leg][motor];
        }
    }
    return mReturnTorque;
}