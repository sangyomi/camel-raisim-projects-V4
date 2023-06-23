//
// Created by jaehoon on 23. 4. 15.
//

#ifndef CAMEL_RAISIM_PROJECTS_QUADTRAJECTORYGENERATOR_HPP
#define CAMEL_RAISIM_PROJECTS_QUADTRAJECTORYGENERATOR_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

class QuadTrajectoryGenerator
{
public:
    QuadTrajectoryGenerator()
    {
        mMatrixA << -2, -4, 2,
        1, 4, -1,
        1, 0, 0;
    }
    void updateTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration);
    void calculateCoefficient();
    double getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(3, 3);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(3, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(3, 1);
    double mReferenceTime;
    double mTimeDuration;
};


#endif //CAMEL_RAISIM_PROJECTS_QUADTRAJECTORYGENERATOR_HPP
