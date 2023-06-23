//
// Created by jaehoon on 23. 5. 18.
//

#ifndef CAMEL_RAISIM_PROJECTS_SWINGLEGLOCAL_HPP
#define CAMEL_RAISIM_PROJECTS_SWINGLEGLOCAL_HPP
#define PNUM 4

#include <cmath>
#include <canine_util/RobotMath.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

class SwingLegLocal
{
public:
    SwingLegLocal();
    void SetParameters();
    void SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg);
    void GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg);
    void GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg);
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


#endif //CAMEL_RAISIM_PROJECTS_SWINGLEGLOCAL_HPP
