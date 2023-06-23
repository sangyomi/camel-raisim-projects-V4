//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULSTATEESTIMATOR_HPP
#define RAISIM_SIMULSTATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotMath.hpp>
#include <canine_util/Filter.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


class SimulStateEstimator{
public:
    SimulStateEstimator(raisim::ArticulatedSystem* robot, RigidBodyDynamics::Model* model);
    void StateEstimatorFunction();

private:
    void doLegKinematics();
    void getContactState();
    void getJointState();
    void getRobotAngulerState();
    void getRobotFootPosition();
    void getRobotLinearState();
    void showOutputs();
    void updateState();
    void updateRBDL();

private:
    const double mDt;
    bool mbContactState[4];

    Eigen::Matrix<double, 18, 18> mGain;

    Eigen::VectorXd mBeta;
    Eigen::VectorXd mCalcTorque;
    Eigen::VectorXd mMomentum;
    Eigen::VectorXd mPrevQ;
    Eigen::VectorXd mPrevQd;
    Eigen::VectorXd mQ;
    Eigen::VectorXd mQd;
    Eigen::VectorXd mQdd;
    Eigen::VectorXd mResidual;
    Eigen::VectorXd mTau;
    Eigen::VectorXd mGeneralizedForce;

    double tempV[3];
    int mMulti;
    int mIteration;
    double mRawEulerAngle[3];

    Mat4<double> mTransMat[4];
    Vec3<double> mGlobalBaseEulerVelocity;
    double mTempGlobalBaseEulerVelocity[3];
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;

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

#endif //RAISIM_SIMULSTATEESTIMATOR_HPP
