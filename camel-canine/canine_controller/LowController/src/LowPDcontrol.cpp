//
// Created by hs on 22. 10. 27.
//

/***
 *  Low Local PD controller
 */
#include <LowController/LowPDcontrol.hpp>

extern pSHM sharedMemory;

LowPDcontrol::LowPDcontrol()
    : mTorqueLimit(30)
    , bIsFirstRunStand{ false, false, false, false }
    , bIsFirstHome{ true, true, true, true }
    , mSwingPgain{ 60, 60, 60 }
    , mSwingDgain{ 5.0, 5.0, 5.0 }
    , mStandPgain{ 20, 20, 20 }
    , mStandDgain{ 5.0, 5.0, 5.0 }
{
    mTorque->setZero();
    mHipPosition[LF_IDX] << HIP_X_POS, HIP_Y_POS, 0;
    mHipPosition[RF_IDX] << HIP_X_POS, -HIP_Y_POS, 0;
    mHipPosition[LB_IDX] << -HIP_X_POS, HIP_Y_POS, 0;
    mHipPosition[RB_IDX] << -HIP_X_POS, -HIP_Y_POS, 0;
    mShoulderPosition[LF_IDX] << SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[RF_IDX] << SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[LB_IDX] << -SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[RB_IDX] << -SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mHip2ShoulderPosition[LF_IDX] = mShoulderPosition[LF_IDX] - mHipPosition[LF_IDX];
    mHip2ShoulderPosition[RF_IDX] = mShoulderPosition[RF_IDX] - mHipPosition[RF_IDX];
    mHip2ShoulderPosition[LB_IDX] = mShoulderPosition[LB_IDX] - mHipPosition[LB_IDX];
    mHip2ShoulderPosition[RB_IDX] = mShoulderPosition[RB_IDX] - mHipPosition[RB_IDX];

    for(int leg=0; leg<4; leg++)
    {
        mCount[leg] = 0;
        mLocomotionDesiredVelocity[leg].setZero();
    }
}

//LowPDcontrol::~LowPDcontrol()
//{
//    delete[] SwingLegTrajectory;
//}

void LowPDcontrol::DoControl()
{
    updateState();
    setLegControl();
    setControlInput();
}

void LowPDcontrol::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
    }

    for (int idx=0; idx<4; idx++)
    {
        mBaseQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
        mBodyFootPosition[idx] = sharedMemory->bodyFootPosition[idx];
        mGlobalFootPosition[idx] = sharedMemory->globalFootPosition[idx];
        for (int mt=0; mt<3; mt++)
        {
            mMotorPosition[idx][mt] = sharedMemory->motorPosition[idx*3+mt];
            mMotorVelocity[idx][mt] = sharedMemory->motorVelocity[idx*3+mt];
        }
    }

    double cy = cos(sharedMemory->baseDesiredEulerPosition[2] * 0.5);
    double sy = sin(sharedMemory->baseDesiredEulerPosition[2] * 0.5);
    double cp = cos(sharedMemory->baseDesiredEulerPosition[1] * 0.5);
    double sp = sin(sharedMemory->baseDesiredEulerPosition[1] * 0.5);
    double cr = cos(sharedMemory->baseDesiredEulerPosition[0] * 0.5);
    double sr = sin(sharedMemory->baseDesiredEulerPosition[0] * 0.5);

    mBaseDesiredQuaternion[0] = cr * cp * cy + sr * sp * sy;
    mBaseDesiredQuaternion[1] = sr * cp * cy - cr * sp * sy;
    mBaseDesiredQuaternion[2] = cr * sp * cy + sr * cp * sy;
    mBaseDesiredQuaternion[3] = cr * cp * sy - sr * sp * cy;

    for(int i = 0 ; i<4 ;i++)
    {
        mHipFootPosition[i] = mBodyFootPosition[i] - mHipPosition[i];
        mGaitTable[i] = sharedMemory->gaitTable[i];
    }

}

void LowPDcontrol::getJointPos(Vec3<double>& jointPos, Vec3<double> desiredHip2FootPos, const int& leg, bool stand)
{
    Vec3<double> localYawPosition;
    Vec3<double> localBasePosition;
    Vec3<double> desiredFootPos;
    Vec3<double> temp;
    Vec4<double> baseLocalDesiredQuaternion;
    Vec3<double> baseLocalDesiredEuler;

    if(stand)
    {
        Mat3<double> yawRot;

        mLocalBaseDesiredLinear[leg][0] -= sharedMemory->baseLocalDesiredVelocity[0] * LOW_CONTROL_dT;
        mLocalBaseDesiredLinear[leg][1] -= sharedMemory->baseLocalDesiredVelocity[1] * LOW_CONTROL_dT;
        mLocalBaseDesiredLinear[leg][2] -= sharedMemory->baseLocalDesiredVelocity[2] * LOW_CONTROL_dT;

        mLocalBaseDesiredEuler[leg][0] -= sharedMemory->baseDesiredEulerVelocity[0] * LOW_CONTROL_dT;
        mLocalBaseDesiredEuler[leg][1] -= sharedMemory->baseDesiredEulerVelocity[1] * LOW_CONTROL_dT;
        mLocalBaseDesiredEuler[leg][2] -= sharedMemory->baseDesiredEulerVelocity[2] * LOW_CONTROL_dT;

        Mat3<double> rot;
        double rc = cos(mLocalBaseDesiredEuler[leg][0]);
        double rs = sin(mLocalBaseDesiredEuler[leg][0]);
        double pc = cos(mLocalBaseDesiredEuler[leg][1]);
        double ps = sin(mLocalBaseDesiredEuler[leg][1]);
        double yc = cos(mLocalBaseDesiredEuler[leg][2]);
        double ys = sin(mLocalBaseDesiredEuler[leg][2]);
        rot << yc*pc, yc*ps*rs - ys*rc, yc*ps*rc+ys*rs,
                ys*pc, ys*ps*rs + yc*rc, ys*ps*rc-yc*rs,
                -ps,    pc*rs,            pc*rc;
        localBasePosition = desiredHip2FootPos + mLocalBaseDesiredLinear[leg];
        localBasePosition[2] = -sharedMemory->baseDesiredPosition[2]-POS_FOO_GND_Z;
        temp = rot * (localBasePosition + mHipPosition[leg]) - mHipPosition[leg];
        desiredFootPos[0] = temp[0];
        desiredFootPos[1] = temp[1];
        desiredFootPos[2] = temp[2];
        sharedMemory->desiredFootPosition[leg] = desiredFootPos + mHipPosition[leg];
    }
    else
    {
        Mat3<double> rot;
        double rc = cos(sharedMemory->baseEulerPosition[0]);
        double rs = sin(sharedMemory->baseEulerPosition[0]);
        double pc = cos(sharedMemory->baseEulerPosition[1]);
        double ps = sin(sharedMemory->baseEulerPosition[1]);
        rot << pc, 0, -ps,
                -rs*ps, rc, -rs*pc,
                rc*ps, rs, rc*pc;
        if(sharedMemory->isRamp)
        {
//            desiredFootPos = (desiredHip2FootPos) - rot * mHipPosition[leg]; //경사 + body 기준
            desiredFootPos = (desiredHip2FootPos + mHipPosition[leg]) - rot *  mHipPosition[leg]; // 경사 + hip 기준 !!!!
        }
        else
        {
//            desiredFootPos = (desiredHip2FootPos) - rot.inverse() * mHipPosition[leg]; // 평지 + body 기준
            desiredFootPos = (desiredHip2FootPos + mHipPosition[leg]) - rot.inverse() *  mHipPosition[leg]; // 평지 + hip 기준 !!!!
        }
        sharedMemory->desiredFootPosition[leg] = desiredFootPos + mHipPosition[leg];
    }
    GetLegInvKinematics(jointPos, desiredFootPos, leg);
}
void LowPDcontrol::getJointVel(Vec3<double>& jointVel, const int& leg, bool stand)
{
    if(stand)
    {
        Mat3<double> jacobian;
        Mat3<double> rot;
        double rc = cos(mLocalBaseDesiredEuler[leg][0]);
        double rs = sin(mLocalBaseDesiredEuler[leg][0]);
        double pc = cos(mLocalBaseDesiredEuler[leg][1]);
        double ps = sin(mLocalBaseDesiredEuler[leg][1]);
        double yc = cos(mLocalBaseDesiredEuler[leg][2]);
        double ys = sin(mLocalBaseDesiredEuler[leg][2]);
        rot << yc*pc, yc*ps*rs - ys*rc, yc*ps*rc+ys*rs,
                ys*pc, ys*ps*rs + yc*rc, ys*ps*rc-yc*rs,
                -ps,    pc*rs,            pc*rc;
        Vec3<double> vel = -sharedMemory->baseDesiredEulerVelocity.cross(rot.inverse()*(mStandFootPosition[leg] + mLocalBaseDesiredLinear[leg] + mHipPosition[leg])) - rot.inverse()*sharedMemory->baseLocalDesiredVelocity;
        GetJacobian2(jacobian, mStandJointPos, leg);
        jointVel = -jacobian.inverse()*vel;
    }
    else
    {
        Mat3<double> jacobian;
        GetJacobian2(jacobian, mSwingJointPos, leg);
        jointVel = -jacobian.inverse()*mSwingFootDesiredVelocity[leg];
    }
    sharedMemory->motorDesiredVelocity[leg * 3] = jointVel[0];
    sharedMemory->motorDesiredVelocity[leg * 3 + 1] = jointVel[1];
    sharedMemory->motorDesiredVelocity[leg * 3 + 2] = jointVel[2];
}
void LowPDcontrol::setLegControl()
{
    SwingLegTrajectory.SetParameters();
    for (int leg = 0; leg < 4; leg++)
    {
        if (mGaitTable[leg] == 0)
        {
            if(bIsRunTrot[leg])
            {
                mLocomotionDesiredVelocity[leg] = sharedMemory->baseLocalDesiredVelocity;
            }
            if(mCount[leg]%1 == 0 & mCount[leg] < 80)
            {
                Mat3<double> yawRateRot;
                Vec3<double> localYawRatePosition;
                Vec3<double> localBaseVelocity;
                Vec3<double> desiredBodyFootPosition;

                double yawRate = sharedMemory->standPeriod/2 * sharedMemory->baseEulerVelocity[2];
                yawRateRot << cos(yawRate), -sin(yawRate), 0,
                        sin(yawRate),  cos(yawRate), 0,
                        0, 0, 1;
                double RaibertV = 0.0*sharedMemory->standPeriod/2; //0.02, 0.005
                localBaseVelocity = GetBaseRotationMatInverse(mBaseQuaternion) * mBaseVelocity;
//                Vec3<double> S = RaibertV*(localBaseVelocity - mLocomotionDesiredVelocity[leg]);
                Vec3<double> S = RaibertV*(localBaseVelocity - sharedMemory->baseLocalDesiredVelocity);
                Vec3<double> V = mGaitTable[LF_IDX]*sharedMemory->bodyFootPosition[LF_IDX] + mGaitTable[RF_IDX]*sharedMemory->bodyFootPosition[RF_IDX]
                                 - mGaitTable[LB_IDX]*sharedMemory->bodyFootPosition[LB_IDX] - mGaitTable[RB_IDX]*sharedMemory->bodyFootPosition[RB_IDX];
                Vec3<double> RaibertPosition =  (S - (S.dot(V))/(V.dot(V))*V);
                //    Vec3<double> RaibertPosition;
                //    if(leg == LF_IDX)
                //    {
                //        RaibertPosition =  (S - (S.dot(V))/(V.dot(V))*V);
                //    }
                //    else
                //    {
                //        RaibertPosition.setZero();
                //    }
//                localYawRatePosition = localBaseVelocity* sharedMemory->standPeriod/2;
//                localYawRatePosition = yawRateRot * (mShoulderPosition[leg] + localYawRatePosition);
//                desiredBodyFootPosition = localYawRatePosition + RaibertPosition;

//                localYawRatePosition = yawRateRot * (mHip2ShoulderPosition[leg] + localBaseVelocity* STAND_PERIOD/2); //!!!!
//                desiredBodyFootPosition = localYawRatePosition + RaibertPosition; //!!!!

                localYawRatePosition = yawRateRot * (mHip2ShoulderPosition[leg] + localBaseVelocity* sharedMemory->standPeriod/2  + RaibertPosition); //!!!!
                desiredBodyFootPosition = localYawRatePosition; //!!!!

                if(bIsRunTrot[leg])
                {
                    SwingLegTrajectory.SetFootTrajectory(mHipFootPosition[leg], desiredBodyFootPosition, leg);
                    bIsRunTrot[leg] = false;
                    bIsFirstRunStand[leg] = true;
                }
                else
                {
                    SwingLegTrajectory.ReplanningXY(desiredBodyFootPosition, leg);
                }
            }
            SwingLegTrajectory.GetPositionTrajectory(mSwingFootPosition[leg], leg);
            SwingLegTrajectory.GetVelocityTrajectory(mSwingFootDesiredVelocity[leg], leg);
            getJointPos(mSwingJointPos, mSwingFootPosition[leg], leg, false);
            getJointVel(mSwingJointVel, leg, false);
            for(int mt=0; mt<3; mt++)
            {
                sharedMemory->pdTorque[leg][mt] = mSwingPgain[mt] * (mSwingJointPos[mt] - mMotorPosition[leg][mt])
                    + mSwingDgain[mt] * (mSwingJointVel[mt] - mMotorVelocity[leg][mt]);
            }
            mCount[leg]++;
        }
        else
        {
            if (bIsFirstHome[leg] == true)
            {
                mStandFootPosition[leg] = mHipFootPosition[leg];
                mStandFootPosition[leg][0] += 0.07;
                mLocalBaseDesiredEuler[leg].setZero();
                mLocalBaseDesiredLinear[leg].setZero();
                bIsFirstHome[leg] = false;
            }
            if(bIsFirstRunStand[leg])
            {
//                mStandFootPosition[leg] = mSwingFootPosition[leg] - mHipPosition[leg]; //first stand foot pos from each hip
                mStandFootPosition[leg] = mSwingFootPosition[leg]; //first stand foot pos from each hip
                bIsFirstRunStand[leg] = false;
                bIsRunTrot[leg] = true;
                if(sharedMemory->isRamp)
                {
                    mLocalBaseDesiredEuler[leg].setZero();
                    sharedMemory->baseDesiredEulerPosition[0] = sharedMemory->baseEulerPosition[0];
                    sharedMemory->baseDesiredEulerPosition[1] = sharedMemory->baseEulerPosition[1];
                    mLocalBaseDesiredEuler[leg][0] = sharedMemory->baseDesiredEulerPosition[0];
                    mLocalBaseDesiredEuler[leg][1] = sharedMemory->baseDesiredEulerPosition[1];
                }
                else
                {
                    mLocalBaseDesiredEuler[leg].setZero(); //평지
                    sharedMemory->baseDesiredEulerPosition[0] = 0.0;
                    sharedMemory->baseDesiredEulerPosition[1] = 0.0;
                }
                mLocalBaseDesiredLinear[leg].setZero();
            }
            bIsRunTrot[leg] = true;
            mCount[leg] = 0;
            getJointPos(mStandJointPos, mStandFootPosition[leg], leg, true);
            getJointVel(mStandJointVel, leg, true);
            for(int mt=0; mt<3; mt++)
            {
                sharedMemory->pdTorque[leg][mt] = mStandPgain[mt] * (mStandJointPos[mt] - mMotorPosition[leg][mt])
                    + mStandDgain[mt] * (mStandJointVel[mt] - mMotorVelocity[leg][mt]);
            }
        }
    }

    for(int idx=0; idx<4; idx++)
    {
        if(!sharedMemory->isNan)
        {
            mTorque[idx][0] = sharedMemory->pdTorque[idx][0] + sharedMemory->mpcTorque[idx][0];
            mTorque[idx][1] = sharedMemory->pdTorque[idx][1] + sharedMemory->mpcTorque[idx][1];
            mTorque[idx][2] = sharedMemory->pdTorque[idx][2] + sharedMemory->mpcTorque[idx][2];
        }
        else
        {
            mTorque[idx].setZero();
        }
    }
}

void LowPDcontrol::setControlInput()
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
            sharedMemory->motorDesiredTorque[leg*3+motor] = mTorque[leg][motor];
        }
    }
}

double* LowPDcontrol::GetTorque()
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