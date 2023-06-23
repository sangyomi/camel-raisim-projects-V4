//
// Created by jaehoon on 23. 5. 18.
//

#include "ControlUtils/SwingLegLocal.hpp"

extern pSHM sharedMemory;

SwingLegLocal::SwingLegLocal()
{
    for(int i = 0; i<4 ; i++)
    {
        px[i].setZero();
        py[i].setZero();
        pz[i] << -0.325, -0.25, -0.25, -0.325;
    }
}

void SwingLegLocal::SetParameters()
{
    mTimeDuration = sharedMemory->swingPeriod;
}

void SwingLegLocal::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    px[leg][0] = initPos[0];
    px[leg][1] = initPos[0] * 3/4 + desiredPos[0] * 1/4 ;
    px[leg][2] = initPos[0] * 1/4 + desiredPos[0] * 3/4;
    px[leg][3] = desiredPos[0];

    py[leg][0] = initPos[1];
    py[leg][1] = initPos[1] * 3/4 + desiredPos[1] * 1/4 ;
    py[leg][2] = initPos[1] * 1/4 + desiredPos[1] * 3/4 ;
    py[leg][3] = desiredPos[1];

    mReferenceTime[leg] = sharedMemory->localTime;
}

double SwingLegLocal::factorial(double value)
{
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
    {
        result *= i;
    }
    return result;
}

void SwingLegLocal::GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg)
{
    double normalizedTime = (sharedMemory->localTime - mReferenceTime[leg]) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX[leg] = 0.0;
    sumY[leg] = 0.0;
    sumZ[leg] = 0.0;

    double coeff;
    for (int i = 0; i < PNUM; i++)
    {
        coeff = factorial(PNUM - 1) / (factorial(i) * factorial(PNUM - 1 - i))
            * pow(normalizedTime, i) * pow((1 - normalizedTime), (PNUM - 1 - i));
        sumX[leg] += coeff * px[leg][i];
        sumY[leg] += coeff * py[leg][i];
        sumZ[leg] += coeff * pz[leg][i];
    }

    desiredPosition[0] = sumX[leg];
    desiredPosition[1] = sumY[leg];
    desiredPosition[2] = sumZ[leg];
}

void SwingLegLocal::GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg)
{
    double normalizedTime = (sharedMemory->localTime - mReferenceTime[leg]) / mTimeDuration;

    desiredVelocity[0] = 3 * pow((1 - normalizedTime), 2) * (px[leg][1] - px[leg][0]) + 6 * (1 - normalizedTime) * normalizedTime * (px[leg][2] - px[leg][1]) + 3 * pow(normalizedTime, 2) * (px[leg][3] - px[leg][2]);
    desiredVelocity[1] = 3 * pow((1 - normalizedTime), 2) * (py[leg][1] - py[leg][0]) + 6 * (1 - normalizedTime) * normalizedTime * (py[leg][2] - py[leg][1]) + 3 * pow(normalizedTime, 2) * (py[leg][3] - py[leg][2]);
    desiredVelocity[2] = 3 * pow((1 - normalizedTime), 2) * (pz[leg][1] - pz[leg][0]) + 6 * (1 - normalizedTime) * normalizedTime * (pz[leg][2] - pz[leg][1]) + 3 * pow(normalizedTime, 2) * (pz[leg][3] - pz[leg][2]);
    desiredVelocity = desiredVelocity/mTimeDuration;
}