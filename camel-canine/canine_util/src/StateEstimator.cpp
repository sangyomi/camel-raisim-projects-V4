//
// Created by hs on 22. 10. 14.
//

#include <canine_util/StateEstimator.hpp>

extern pSHM sharedMemory;

CanineFilter::Vec3MAF vecMAF(20,20,10);

StateEstimator::StateEstimator(RigidBodyDynamics::Model* model)
    : mDt(ESTIMATOR_dT)
    , mGain(Eigen::Matrix<double,18,18>::Identity() * 50)
    , mModel(model)
    , mbIsFirstRun(true)
    , mAlphaStep(25)
{
    for(int i=0; i<4; i++)
    {
        mbContactState[i] = true;
        mbIsFirstCon[i] = true;
    }
    mBeta = Eigen::VectorXd::Zero(18);
    mCalcTorque = Eigen::VectorXd::Zero(18);
    mH = Eigen::MatrixXd::Zero(18,18);
    mMomentum = Eigen::VectorXd::Zero(18);
    mPrevH = Eigen::MatrixXd::Zero(18,18);
    mPrevQ = Eigen::VectorXd::Zero(19);
    mPrevQd = Eigen::VectorXd::Zero(18);
    mQ = Eigen::VectorXd::Zero(19);
    mQd = Eigen::VectorXd::Zero(18);
    mResidual = Eigen::VectorXd::Zero(18);
    mTau = Eigen::VectorXd::Zero(18);

    /// leg kinematics
    mGlobalBasePos << 0.0, 0.0, 0.076;
    mGlobalBaseVel << 0.0, 0.0, 0.0;
}

void StateEstimator::StateEstimatorFunction()
{
    if (sharedMemory->motorLFState && sharedMemory->motorRFState && sharedMemory->motorLBState && sharedMemory->motorRBState)
    {
        updateState();
        updateRBDL();
        getRobotFootPosition();
        getContactState();
        getRobotLinearState();
    }
}

void StateEstimator::doLegKinematics()
{
    Mat3<double> rotWorld2Body;
    Mat3<double> jacobian;
    Vec3<double> bodyFootPos[4];
    Vec3<double> globalBaseVel[4];
    Vec3<double> motorPositions[4];
    Vec3<double> jointVelocities[4];
    Vec3<double> angularVel;

    for(int idx=0; idx<4; idx++)
    {
        bodyFootPos[idx] = sharedMemory->bodyFootPosition[idx];

        motorPositions[idx][0] = sharedMemory->motorPosition[idx*3];
        motorPositions[idx][1] = sharedMemory->motorPosition[idx*3 + 1];
        motorPositions[idx][2] = sharedMemory->motorPosition[idx*3 + 2];

        jointVelocities[idx][0] = sharedMemory->motorVelocity[idx*3];
        jointVelocities[idx][1] = sharedMemory->motorVelocity[idx*3 + 1];
        jointVelocities[idx][2] = sharedMemory->motorVelocity[idx*3 + 2];
    }

    rotWorld2Body = GetBaseRotationMat(sharedMemory->baseQuartPosition);

    angularVel[0] = sharedMemory->baseEulerVelocity[0];
    angularVel[1] = sharedMemory->baseEulerVelocity[1];
    angularVel[2] = sharedMemory->baseEulerVelocity[2];

    if(mbIsFirstRun)
    {
        mGlobalContactFootPos[LF_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[LF_IDX];
        mGlobalContactFootPos[RF_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[RF_IDX];
        mGlobalContactFootPos[LB_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[LB_IDX];
        mGlobalContactFootPos[RB_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[RB_IDX];

        mbIsFirstRun = false;
    }

    if (mbContactState[LF_IDX] & mbContactState[RF_IDX] & mbContactState[LB_IDX] & mbContactState[RB_IDX])
    {
        if(!mbIsFirstCon[LF_IDX] & mbIsFirstCon[RF_IDX])
        {
            mGlobalBasePos = (mGlobalContactFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX]
                            + mGlobalContactFootPos[RB_IDX] - rotWorld2Body * bodyFootPos[RB_IDX])/2;
            mGlobalBasePos[2] = -(rotWorld2Body * bodyFootPos[LF_IDX])[2];
            GetJacobian2(jacobian,motorPositions[LF_IDX],LF_IDX);
            globalBaseVel[LF_IDX] = rotWorld2Body * GetSkew(bodyFootPos[LF_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[LF_IDX];
            GetJacobian2(jacobian,motorPositions[RB_IDX],RB_IDX);
            globalBaseVel[RB_IDX] = rotWorld2Body * GetSkew(bodyFootPos[RB_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[RB_IDX];
            mGlobalBaseVel = (globalBaseVel[LF_IDX] + globalBaseVel[RB_IDX])/2;

            mGlobalBaseVelBuff[LF_IDX] = globalBaseVel[LF_IDX];
            mGlobalBaseVelBuff[RB_IDX] = globalBaseVel[RB_IDX];
        }
        else if(mbIsFirstCon[LF_IDX] & !mbIsFirstCon[RF_IDX])
        {
            mGlobalBasePos = (mGlobalContactFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX]
                            + mGlobalContactFootPos[LB_IDX] - rotWorld2Body * bodyFootPos[LB_IDX])/2;
            mGlobalBasePos[2] = -(rotWorld2Body * bodyFootPos[RF_IDX])[2];
            GetJacobian2(jacobian,motorPositions[RF_IDX],RF_IDX);
            globalBaseVel[RF_IDX] = rotWorld2Body * GetSkew(bodyFootPos[RF_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[RF_IDX];
            GetJacobian2(jacobian,motorPositions[LB_IDX],LB_IDX);
            globalBaseVel[LB_IDX] = rotWorld2Body * GetSkew(bodyFootPos[LB_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[LB_IDX];
            mGlobalBaseVel = (globalBaseVel[RF_IDX] + globalBaseVel[LB_IDX])/2;

            mGlobalBaseVelBuff[RF_IDX] = globalBaseVel[RF_IDX];
            mGlobalBaseVelBuff[LB_IDX] = globalBaseVel[LB_IDX];
        }
        else
        {
            mGlobalBasePos = (mGlobalContactFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX]
                            + mGlobalContactFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX]
                            + mGlobalContactFootPos[LB_IDX] - rotWorld2Body * bodyFootPos[LB_IDX]
                            + mGlobalContactFootPos[RB_IDX] - rotWorld2Body * bodyFootPos[RB_IDX])/4;
            mGlobalBasePos[2] = -((rotWorld2Body * bodyFootPos[LB_IDX])[2] + (rotWorld2Body * bodyFootPos[RB_IDX])[2])/2;

            GetJacobian2(jacobian,motorPositions[LF_IDX],LF_IDX);
            globalBaseVel[LF_IDX] = rotWorld2Body * GetSkew(bodyFootPos[LF_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[LF_IDX];
            GetJacobian2(jacobian,motorPositions[RF_IDX],RF_IDX);
            globalBaseVel[RF_IDX] = rotWorld2Body * GetSkew(bodyFootPos[RF_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[RF_IDX];
            GetJacobian2(jacobian,motorPositions[LB_IDX],LB_IDX);
            globalBaseVel[LB_IDX] = rotWorld2Body * GetSkew(bodyFootPos[LB_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[LB_IDX];
            GetJacobian2(jacobian,motorPositions[RB_IDX],RB_IDX);
            globalBaseVel[RB_IDX] = rotWorld2Body * GetSkew(bodyFootPos[RB_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[RB_IDX];

            mGlobalBaseVel = (globalBaseVel[LF_IDX] + globalBaseVel[RF_IDX] + globalBaseVel[LB_IDX] + globalBaseVel[RB_IDX])/4;

            mGlobalBaseVelBuff[LF_IDX] = globalBaseVel[LF_IDX];
            mGlobalBaseVelBuff[RF_IDX] = globalBaseVel[RF_IDX];
            mGlobalBaseVelBuff[LB_IDX] = globalBaseVel[LB_IDX];
            mGlobalBaseVelBuff[RB_IDX] = globalBaseVel[RB_IDX];
        }
    }
    else if (mbContactState[LF_IDX])
    {
        if (mbIsFirstCon[LF_IDX]) {
            mGlobalContactFootPos[LF_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[LF_IDX];
            mGlobalContactFootPos[RB_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[RB_IDX];
            mbIsFirstCon[LF_IDX] = false;
            mbIsFirstCon[RF_IDX] = true;
            mAlpha = 0;
        }
        mGlobalBasePos = mGlobalContactFootPos[LF_IDX] - rotWorld2Body * bodyFootPos[LF_IDX];
        mGlobalBasePos[2] = -(rotWorld2Body * bodyFootPos[LF_IDX])[2];
        GetJacobian2(jacobian,motorPositions[LF_IDX],LF_IDX);
        globalBaseVel[LF_IDX] = rotWorld2Body * GetSkew(bodyFootPos[LF_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[LF_IDX];
        GetJacobian2(jacobian,motorPositions[RB_IDX],RB_IDX);
        globalBaseVel[RB_IDX] = rotWorld2Body * GetSkew(bodyFootPos[RB_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[RB_IDX];
        mGlobalBaseVel = ((1-mAlpha/mAlphaStep) * (mGlobalBaseVelBuff[RF_IDX] + mGlobalBaseVelBuff[LB_IDX]) + (mAlpha/mAlphaStep) * (globalBaseVel[LF_IDX] + globalBaseVel[RB_IDX]))/2;

        mGlobalBaseVelBuff[LF_IDX] = globalBaseVel[LF_IDX];
        mGlobalBaseVelBuff[RB_IDX] = globalBaseVel[RB_IDX];
        if(mAlpha<mAlphaStep)
        {
            mAlpha++;
        }
    }
    else if (mbContactState[RF_IDX])
    {
        if (mbIsFirstCon[RF_IDX]) {
            mGlobalContactFootPos[RF_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[RF_IDX];
            mGlobalContactFootPos[LB_IDX] = mGlobalBasePos + rotWorld2Body * bodyFootPos[LB_IDX];
            mbIsFirstCon[RF_IDX] = false;
            mbIsFirstCon[LF_IDX] = true;
            mAlpha = 0;
        }
        mGlobalBasePos = mGlobalContactFootPos[RF_IDX] - rotWorld2Body * bodyFootPos[RF_IDX];
        mGlobalBasePos[2] = -(rotWorld2Body * bodyFootPos[RF_IDX])[2];
        GetJacobian2(jacobian,motorPositions[RF_IDX],RF_IDX);
        globalBaseVel[RF_IDX] = rotWorld2Body * GetSkew(bodyFootPos[RF_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[RF_IDX];
        GetJacobian2(jacobian,motorPositions[LB_IDX],LB_IDX);
        globalBaseVel[LB_IDX] = rotWorld2Body * GetSkew(bodyFootPos[LB_IDX]) * angularVel + rotWorld2Body * jacobian * jointVelocities[LB_IDX];
        mGlobalBaseVel = ((1-mAlpha/mAlphaStep) * (mGlobalBaseVelBuff[LF_IDX] + mGlobalBaseVelBuff[RB_IDX]) + (mAlpha/mAlphaStep) * (globalBaseVel[RF_IDX] + globalBaseVel[LB_IDX]))/2;

        mGlobalBaseVelBuff[RF_IDX] = globalBaseVel[RF_IDX];
        mGlobalBaseVelBuff[LB_IDX] = globalBaseVel[LB_IDX];
        if(mAlpha<mAlphaStep)
        {
            mAlpha ++;
        }
    }

    mGlobalBaseVel = vecMAF.GetFilteredVar(mGlobalBaseVel);
}

void StateEstimator::getContactState()
{
    mTau.setZero();
    double res[4];
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(* mModel, mQ, mH, true);
    RigidBodyDynamics::InverseDynamics(*mModel, mQ, mQd, Eigen::VectorXd::Zero(18),mTau);

    mBeta = mTau - (mH - mPrevH) / mDt * mQd;
    mMomentum = mMomentum + mCalcTorque * mDt - mBeta * mDt + mResidual * mDt;
    mResidual = mGain * (-mMomentum + mH * mQd);

    res[LF_IDX] = sqrt(pow(mResidual[6],2) + pow(mResidual[7],2) + pow(mResidual[8],2));
    res[RF_IDX] = sqrt(pow(mResidual[9],2) + pow(mResidual[10],2) + pow(mResidual[11],2));
    res[LB_IDX] = sqrt(pow(mResidual[12],2) + pow(mResidual[13],2) + pow(mResidual[14],2));
    res[RB_IDX] = sqrt(pow(mResidual[15],2) + pow(mResidual[16],2) + pow(mResidual[17],2));

    for(int i=0; i<4; i++)
    {
        sharedMemory->tempRes[i] = res[i];
    }

    mPrevH = mH;
    mPrevQ = mQ;
    mPrevQd = mQd;

    if(mbContactState[LF_IDX] == 1 && res[LF_IDX] < 3)
    {
        mbContactState[LF_IDX] = 0;
    }
    else if(mbContactState[LF_IDX] == 0 && res[LF_IDX] > 12)
    {
        mbContactState[LF_IDX] = 1;
    }

    if(mbContactState[RF_IDX] == 1 && res[RF_IDX] < 3)
    {
        mbContactState[RF_IDX] = 0;
    }
    else if(mbContactState[RF_IDX] == 0 && res[RF_IDX] > 5)
    {
        mbContactState[RF_IDX] = 1;
    }

    if(mbContactState[LB_IDX] == 1 && res[LB_IDX] < 3)
    {
        mbContactState[LB_IDX] = 0;
    }
    else if(mbContactState[LB_IDX] == 0 && res[LB_IDX] >5)
    {
        mbContactState[LB_IDX] = 1;
    }

    if(mbContactState[RB_IDX] == 1 && res[RB_IDX] < 4)
    {
        mbContactState[RB_IDX] = 0;
    }
    else if(mbContactState[RB_IDX] == 0 && res[RB_IDX] >7)
    {
        mbContactState[RB_IDX] = 1;
    }

    for(int i=0; i<4; i++)
    {
        sharedMemory->contactState[i] = mbContactState[i];
        mbContactState[i] = sharedMemory->gaitTable[i];
    }
//    std::cout <<"[FL] : "
//              << std::setw(10) << sharedMemory->simulContactForceFL
//              << std::setw(12) << sharedMemory->tempRes[0]
//              <<" [FR] : "
//              << std::setw(10) << sharedMemory->simulContactForceFR
//              << std::setw(12) << sharedMemory->tempRes[1]
//              <<" [HL] : "
//              << std::setw(10) << sharedMemory->simulContactForceHL
//              << std::setw(12) << sharedMemory->tempRes[2]
//              <<" [HR] : "
//              << std::setw(10) << sharedMemory->simulContactForceHR
//              << std::setw(12) << sharedMemory->tempRes[3]
//              << std::endl;
//
//    std::cout <<"[FL] : "
//              << std::setw(3) << sharedMemory->gaitTable[0]
//              << std::setw(3) << sharedMemory->contactState[0]
//              <<" [FR] : "
//              << std::setw(3) << sharedMemory->gaitTable[1]
//              << std::setw(3) << sharedMemory->contactState[1]
//              <<" [HL] : "
//              << std::setw(3) << sharedMemory->gaitTable[2]
//              << std::setw(3) << sharedMemory->contactState[2]
//              <<" [HR] : "
//              << std::setw(3) << sharedMemory->gaitTable[3]
//              << std::setw(3) << sharedMemory->contactState[3]
//              << std::endl;
}

void StateEstimator::getRobotFootPosition()
{
    TransMatBody2Foot(&mTransMat[0], LF_IDX,
                      sharedMemory->motorPosition[0],
                      sharedMemory->motorPosition[1],
                      sharedMemory->motorPosition[2]);
    TransMatBody2Foot(&mTransMat[1], RF_IDX,
                      sharedMemory->motorPosition[3],
                      sharedMemory->motorPosition[4],
                      sharedMemory->motorPosition[5]);
    TransMatBody2Foot(&mTransMat[2], LB_IDX,
                      sharedMemory->motorPosition[6],
                      sharedMemory->motorPosition[7],
                      sharedMemory->motorPosition[8]);
    TransMatBody2Foot(&mTransMat[3], RB_IDX,
                      sharedMemory->motorPosition[9],
                      sharedMemory->motorPosition[10],
                      sharedMemory->motorPosition[11]);

    Mat3<double> Rot = GetBaseRotationMat(sharedMemory->baseQuartPosition);
    for (int leg=0; leg<4; leg++)
    {
        sharedMemory->bodyFootPosition[leg] = mTransMat[leg].block(0,3,3,1);
        sharedMemory->globalFootPosition[leg] = sharedMemory->basePosition + Rot*sharedMemory->bodyFootPosition[leg];
    }
}

void StateEstimator::getRobotLinearState()
{
    this->doLegKinematics();

    sharedMemory->basePosition = mGlobalBasePos;
    sharedMemory->baseVelocity = mGlobalBaseVel;
}

void StateEstimator::updateRBDL()
{
    mQ[0] = sharedMemory->basePosition[0];
    mQ[1] = sharedMemory->basePosition[1];
    mQ[2] = sharedMemory->basePosition[2];

    mQ[3] = sharedMemory->baseQuartPosition[1]; /// q_x
    mQ[4] = sharedMemory->baseQuartPosition[2]; /// q_y
    mQ[5] = sharedMemory->baseQuartPosition[3]; /// q_z
    mQ[18] = sharedMemory->baseQuartPosition[0]; /// q_w

    mQ[6] = sharedMemory->motorPosition[0];
    mQ[7] = sharedMemory->motorPosition[1];
    mQ[8] = sharedMemory->motorPosition[2];

    mQ[9] = sharedMemory->motorPosition[3];
    mQ[10] = sharedMemory->motorPosition[4];
    mQ[11] = sharedMemory->motorPosition[5];

    mQ[12] = sharedMemory->motorPosition[6];
    mQ[13] = sharedMemory->motorPosition[7];
    mQ[14] = sharedMemory->motorPosition[8];

    mQ[15] = sharedMemory->motorPosition[9];
    mQ[16] = sharedMemory->motorPosition[10];
    mQ[17] = sharedMemory->motorPosition[11];

    mQd[0] = sharedMemory->baseVelocity[0]; /// base_TX
    mQd[1] = sharedMemory->baseVelocity[1]; /// base_TY
    mQd[2] = sharedMemory->baseVelocity[2]; /// base_TZ

    mQd[3] = sharedMemory->baseEulerVelocity[2]; /// base_RZ
    mQd[4] = sharedMemory->baseEulerVelocity[1]; /// base_RY
    mQd[5] = sharedMemory->baseEulerVelocity[0]; /// base_RX

    mQd[6] = sharedMemory->motorVelocity[0]; /// FL_hip_RX
    mQd[7] = sharedMemory->motorVelocity[1];/// FL_thigh_RY
    mQd[8] = sharedMemory->motorVelocity[2];/// FL_calf_RY

    mQd[9] = sharedMemory->motorVelocity[3]; /// FR_hip_RX
    mQd[10] = sharedMemory->motorVelocity[4];/// FR_thigh_RY
    mQd[11] = sharedMemory->motorVelocity[5];/// FR_calf_RY

    mQd[12] = sharedMemory->motorVelocity[6];/// HL_hip_RX
    mQd[13] = sharedMemory->motorVelocity[7];/// HL_thigh_RX
    mQd[14] = sharedMemory->motorVelocity[8];/// HL_calf_RX

    mQd[15] = sharedMemory->motorVelocity[9];/// HR_hip_RX
    mQd[16] = sharedMemory->motorVelocity[10];/// HR_thigh_RX
    mQd[17] = sharedMemory->motorVelocity[11];/// HR_calf_RX

    mCalcTorque[0] = 0;
    mCalcTorque[1] = 0;
    mCalcTorque[2] = 0;

    mCalcTorque[3] = 0;
    mCalcTorque[4] = 0;
    mCalcTorque[5] = 0;

    mCalcTorque[6] = sharedMemory->motorDesiredTorque[0];
    mCalcTorque[7] = sharedMemory->motorDesiredTorque[1];
    mCalcTorque[8] = sharedMemory->motorDesiredTorque[2];

    mCalcTorque[9] = sharedMemory->motorDesiredTorque[3];
    mCalcTorque[10] = sharedMemory->motorDesiredTorque[4];
    mCalcTorque[11] = sharedMemory->motorDesiredTorque[5];

    mCalcTorque[12] = sharedMemory->motorDesiredTorque[6];
    mCalcTorque[13] = sharedMemory->motorDesiredTorque[7];
    mCalcTorque[14] = sharedMemory->motorDesiredTorque[8];

    mCalcTorque[15] = sharedMemory->motorDesiredTorque[9];
    mCalcTorque[16] = sharedMemory->motorDesiredTorque[10];
    mCalcTorque[17] = sharedMemory->motorDesiredTorque[11];
}

void StateEstimator::updateState()
{
    for(int idx=0; idx<4; idx++)
    {
        mQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
    }
    for(int idx=0; idx<3; idx++)
    {
        mAcceleration[idx] = sharedMemory->baseAcceleration[idx];
    }
}