//
// Created by jaehoon on 23. 4. 14.
//

#ifndef CAMEL_RAISIM_PROJECTS_CUBICSWINGLEG_HPP
#define CAMEL_RAISIM_PROJECTS_CUBICSWINGLEG_HPP

#include <cmath>
#include <camel-tools/trajectory.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>

class CubicSwingLeg{
public:
    CubicSwingLeg();
    void SetParameters();
    void SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg);
    void GetTrajectory(Vec3<double>& desiredPosition, const Vec3<double>& initPos, const int& leg);
    void GetVelocityTrajectory(double localTime, Vec3<double>& desiredVelocity, const int& leg);

private:
    CubicTrajectoryGenerator mCubicFootTrajectoryGenerator[12];
    bool mbIsNewTrajectory[4];
    double mReferenceTime[4];
    double mTimeDuration;

    double mLiftUpDuration;
    double mLiftUpHeight[4];
    double mTargetHeight[4];
};


#endif //CAMEL_RAISIM_PROJECTS_CUBICSWINGLEG_HPP
