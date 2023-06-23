//
// Created by jaehoon on 23. 4. 15.
//

#include "../include/QuadTrajectoryGenerator.hpp"

void QuadTrajectoryGenerator::updateTrajectory(double currentPosition, double goalPosition, double currentTime, double timeDuration) {
    mFunctionValue << currentPosition, goalPosition, currentPosition;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void QuadTrajectoryGenerator::calculateCoefficient() {
    mMatrixA << -2, -4, 2,
        1, 4, -1,
        1, 0, 0;
    mCoefficient = mMatrixA * mFunctionValue;
}

double QuadTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return mCoefficient(0,0) * pow(normalizedTime, 2.0) + mCoefficient(1,0) * pow(normalizedTime, 1.0) + mCoefficient(2,0);
}

double QuadTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (2.0 *mCoefficient(0,0) * normalizedTime + mCoefficient(1,0)) / mTimeDuration;
}