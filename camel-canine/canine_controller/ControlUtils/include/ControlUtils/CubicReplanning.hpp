#ifndef CAMEL_RAISIM_PROJECTS_CUBICREPLANNING_HPP
#define CAMEL_RAISIM_PROJECTS_CUBICREPLANNING_HPP

#include <Eigen/Eigen>

class CubicReplanning
{
public:
    CubicReplanning();
    void updateTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration);
    void Replanning(double goalPosition, double currentTime);
    void calculateCoefficient();
    double getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);
    double getAccelerationTrajectory(double currentTime);

private:
    void updateMatrixA(double currentTime);

    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(4, 1);
    double mReferenceTime;
    double mTimeDuration;
};

#endif //CAMEL_RAISIM_PROJECTS_CUBICREPLANNING_HPP
