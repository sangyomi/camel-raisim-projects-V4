//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "Filter.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

class StateEstimator{
public:
    StateEstimator(RigidBodyDynamics::Model* model);
    void StateEstimatorFunction();

private:
    void doLegKinematics();
    void getContactState();
    void getRobotFootPosition();
    void getRobotLinearState();
    void updateState();
    void updateRBDL();

private:
    const double mDt;
    bool mbContactState[4];

    Vec3<double> mAcceleration;
    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];

    Eigen::Matrix<double, 18, 18> mGain;

    Eigen::VectorXd mBeta;
    Eigen::VectorXd mCalcTorque;
    Eigen::VectorXd mMomentum;
    Eigen::VectorXd mPrevQ;
    Eigen::VectorXd mPrevQd;
    Eigen::VectorXd mQ;
    Eigen::VectorXd mQd;
    Eigen::VectorXd mResidual;
    Eigen::VectorXd mTau;

    RigidBodyDynamics::Math::MatrixNd mH;
    RigidBodyDynamics::Math::MatrixNd mPrevH;
    RigidBodyDynamics::Model* mModel;

    /// leg kinematics
    bool mbIsFirstRun;
    bool mbIsFirstCon[4];

    int mAlpha;
    const int mAlphaStep;

    Vec3<double> mGlobalBasePos;
    Vec3<double> mGlobalBaseVel;
    Vec3<double> mGlobalBaseVelBuff[4];
    Vec3<double> mGlobalContactFootPos[4];
    Vec3<double> mPrevGlobalBasePos;
};

#endif //RAISIM_STATEESTIMATOR_HPP
