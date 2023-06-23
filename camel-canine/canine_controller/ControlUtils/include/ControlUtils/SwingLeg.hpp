//
// Created by hs on 22. 10. 24.
//

#ifndef RAISIM_SWINGLEG_HPP
#define RAISIM_SWINGLEG_HPP
#define PNUM 4

#include <cmath>
#include <iostream>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotDescription.hpp>
#include "canine_util/SharedMemory.hpp"
#include "canine_util/RobotMath.hpp"

class SwingLeg{
public:
    SwingLeg();
    void SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg);
    void GetTrajectory(Vec3<double>& desiredPosition, const Vec3<double>& initPos, const int& leg);
    void GetVelocityTrajectory(double currentTime, Vec3<double>& desiredVelocity, const int& leg);

private:
    double factorial(double value);
private:
    double mReferenceTime[4];
    double mTimeDuration;

    double sumX[4];
    double sumY[4];
    double sumZ[4];

    Vec4<double> px[PNUM];
    Vec4<double> py[PNUM];
    Vec4<double> pz[PNUM];
};

#endif //RAISIM_SWINGLEG_HPP
