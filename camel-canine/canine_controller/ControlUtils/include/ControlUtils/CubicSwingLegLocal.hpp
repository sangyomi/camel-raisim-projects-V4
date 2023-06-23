//
// Created by jaehoon on 23. 5. 21.
//

#ifndef CAMEL_RAISIM_PROJECTS_CUBICSWINGLEGLOCAL_HPP
#define CAMEL_RAISIM_PROJECTS_CUBICSWINGLEGLOCAL_HPP

#include <cmath>
#include <camel-tools/trajectory.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include "CubicReplanning.hpp"

class CubicSwingLegLocal{
public:
    CubicSwingLegLocal();
    void SetParameters();
    void SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg);
    void GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg);
    void GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg);
    void ReplanningXY(Vec3<double>& desiredPosition, const int& leg);
private:
    CubicReplanning mCubicFootTrajectoryGenerator[12];
    bool mbIsNewTrajectory[4];
    double mReferenceTime[4];
    double mTimeDuration;

    double mLiftUpDuration;
    double mLiftUpHeight[4];
    double mTargetHeight[4];
};

#endif //CAMEL_RAISIM_PROJECTS_CUBICSWINGLEGLOCAL_HPP
