//
// Created by cha on 22. 10. 5.
//

#ifndef RAISIM_LORDIMU3DMGX5AHRS_HPP
#define RAISIM_LORDIMU3DMGX5AHRS_HPP

#include <iostream>
#include <string>
#include "mscl/mscl.h"


class LordImu3DmGx5Ahrs
{
public:
    LordImu3DmGx5Ahrs(mscl::InertialNode* node);

    void SetConfig(int samplingHz);
    void ParseData();
    void GetCurrentConfig(mscl::InertialNode node);
    double* GetQuaternion();
    double* GetEulerAngle();
    double* GetAcceleration();
    double* GetStabilizedAcceleration();
    double* GetAngularVelocity();
    double* GetLinearAcceleration();
    double GetHeading();


private:
    mscl::InertialNode* mNode;
    std::string mOrientQuaternion;

    ///quaternion
    double mQuaternion[4];

    /// ///mEulerAngle[0] : roll, mEulerAngle[1] : pitch, mEulerAngle[2] : yaw
    double mEulerAngle[3];

    ///all acceleration(from imu channel, unit : g)
    double mAcceleration[3];

    ///all stabilized acceleration(from imu channel, unit : g)
    double mStabilizedAcceleration[3];

    ///linear acceleration(from filter channel, unit : m/s^2)
    double mLinearAcceleration[3];

    ///angular Velocity (unit : rad/s)
    double mAngularVelocity[3];

    ///heading (unit : rad)
    double mHeading;

};


#endif //RAISIM_LORDIMU3DMGX5AHRS_HPP
