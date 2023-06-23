//
// Created by hs on 22. 10. 24.
//
#include <ControlUtils/SwingLeg.hpp>

extern pSHM sharedMemory;

SwingLeg::SwingLeg()
{
    for(int i = 0; i<PNUM ; i++)
    {
        px[i].setZero();
        py[i].setZero();
    }

    pz[0] << 0.0, 0.0, 0.0, 0.0;
    pz[1] << 0.125,0.125,0.125,0.125;
    pz[2] << 0.125, 0.125, 0.125, 0.125;
    pz[3] << 0.0, 0.0, 0.0, 0.0;
}

void SwingLeg::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    mReferenceTime[leg] = sharedMemory->localTime;

    double targetHeight = 0.1;

    px[0][leg] = initPos[0];
    px[1][leg] = initPos[0] + (desiredPos[0] - initPos[0]) * 1/4 ;
    px[2][leg] = initPos[0] + (desiredPos[0] - initPos[0]) * 3/4;
    px[3][leg] = desiredPos[0];

    py[0][leg] = initPos[1];
    py[1][leg] = initPos[1] + (desiredPos[1] - initPos[1]) * 1/4 ;
    py[2][leg] = initPos[1] + (desiredPos[1] - initPos[1]) * 3/4;
    py[3][leg] = desiredPos[1];

    pz[0][leg] = initPos[2];
    pz[1][leg] = initPos[2] + targetHeight;
    pz[2][leg] = initPos[2] + targetHeight;
    pz[3][leg] = initPos[2];
}

double SwingLeg::factorial(double value)
{
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
    {
        result *= i;
    }
    return result;
}

void SwingLeg::GetTrajectory(Vec3<double>& desiredPosition, const Vec3<double>& initPos, const int& leg)
{
    double normalizedTime = (sharedMemory->localTime - mReferenceTime[leg]) / sharedMemory->swingPeriod;
    sumX[leg] = 0.0;
    sumY[leg] = 0.0;
    sumZ[leg] = 0.0;

    double coeff;
    for (int i = 0; i < PNUM; i++)
    {
        coeff = factorial(PNUM - 1) / (factorial(i) * factorial(PNUM - 1 - i))
            * pow(normalizedTime, i) * pow((1 - normalizedTime), (PNUM - 1 - i));
        sumX[leg] += coeff * px[i][leg];
        sumY[leg] += coeff * py[i][leg];
        sumZ[leg] += coeff * pz[i][leg];
    }

    desiredPosition[0] = sumX[leg];
    desiredPosition[1] = sumY[leg];
    desiredPosition[2] = sumZ[leg];
}

void SwingLeg::GetVelocityTrajectory(double currentTime, Vec3<double>& desiredVelocity, const int& leg)
{
    double normalizedTime = (currentTime - mReferenceTime[leg]) / sharedMemory->swingPeriod;

    desiredVelocity[0] = 3 * pow((1 - normalizedTime), 2) * (px[1][leg] - px[0][leg]) + 6 * (1 - normalizedTime) * normalizedTime * (px[2][leg] - px[1][leg]) + 3 * pow(normalizedTime, 2) * (px[3][leg] - px[2][leg]);
    desiredVelocity[1] = 3 * pow((1 - normalizedTime), 2) * (py[1][leg] - py[0][leg]) + 6 * (1 - normalizedTime) * normalizedTime * (py[2][leg] - py[1][leg]) + 3 * pow(normalizedTime, 2) * (py[3][leg] - py[2][leg]);
    desiredVelocity[2] = 3 * pow((1 - normalizedTime), 2) * (pz[1][leg] - pz[0][leg]) + 6 * (1 - normalizedTime) * normalizedTime * (pz[2][leg] - pz[1][leg]) + 3 * pow(normalizedTime, 2) * (pz[3][leg] - pz[2][leg]);
}
