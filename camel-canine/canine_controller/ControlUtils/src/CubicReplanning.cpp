# include "../include/ControlUtils/CubicReplanning.hpp"

CubicReplanning::CubicReplanning()
{
    updateMatrixA(0);
}

void CubicReplanning::updateTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration)
{
    updateMatrixA(0);
    mFunctionValue << currentPosition, goalPosition, 0.0, 0.0;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void CubicReplanning::Replanning(double goalPosition, double currentTime)
{
    double currentPosition = getPositionTrajectory(currentTime);
    double currentVelocity = getVelocityTrajectory(currentTime);
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    updateMatrixA(normalizedTime);
    mFunctionValue << currentPosition, goalPosition, currentVelocity*mTimeDuration, 0;
    calculateCoefficient();
}

void CubicReplanning::calculateCoefficient()
{
    mCoefficient = mMatrixA * mFunctionValue;
}

double CubicReplanning::getPositionTrajectory(double currentTime)
{
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0) + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);
}

double CubicReplanning::getVelocityTrajectory(double currentTime)
{
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (3.0 *mCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1,0) * normalizedTime + mCoefficient(2,0)) / mTimeDuration;
}

double CubicReplanning::getAccelerationTrajectory(double currentTime)
{
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
}

void CubicReplanning::updateMatrixA(double currentTime)
{
    mMatrixA << pow(currentTime, 3), pow(currentTime, 2), currentTime, 1,
        1, 1, 1, 1,
        3*pow(currentTime, 2), 2*currentTime, 1, 0,
        3, 2, 1, 0;
    mMatrixA = mMatrixA.inverse();
}