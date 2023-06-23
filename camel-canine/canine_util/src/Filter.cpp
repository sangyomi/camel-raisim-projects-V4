//
// Created by hs on 22. 11. 1.
//
#include <canine_util/Filter.hpp>

CanineFilter::Vec3LPF::Vec3LPF(double DT, double cutoffFreq)
        : mbIsFirstRun(true)
        , mDT(DT)
        , mCutoffFreq(cutoffFreq)
        , mAlpha(2 * 3.141592 * mCutoffFreq * mDT / (2 * 3.141592 * mCutoffFreq * mDT + 1))
{
    mInputData.setZero();
    mPreviousData.setZero();
    mFilteredData.setZero();
}

Vec3<double> CanineFilter::Vec3LPF::GetFilteredVar(const Vec3<double>& data)
{
    for (int idx=0; idx<3; idx++)
    {
        mInputData[idx] = data[idx];
    }
    doFiltering();
    return mFilteredData;
}

void CanineFilter::Vec3LPF::doFiltering()
{
    if (mbIsFirstRun == true)
    {
        for (int idx=0; idx<3; idx++)
        {
            mPreviousData[idx] = mInputData[idx];
        }
        mbIsFirstRun = false;
    }

    for (int idx=0; idx<3; idx++)
    {
        mFilteredData[idx] = mAlpha * mInputData[idx] + (1 - mAlpha) * mPreviousData[idx];
        mPreviousData[idx] = mFilteredData[idx];
    }
}


CanineFilter::LPF::LPF(double DT, double cutoffFreq)
        : mbIsFirstRun(true)
        , mInputData(0.0)
        , mDT(DT)
        , mCutoffFreq(cutoffFreq)
        , mAlpha(2 * 3.141592 * mCutoffFreq * mDT / (2 * 3.141592 * mCutoffFreq * mDT + 1))
{
}

double CanineFilter::LPF::GetFilteredVar(const double data)
{
    mInputData = data;
    doFiltering();
    return mFilteredData;
}

void CanineFilter::LPF::doFiltering()
{
    if (mbIsFirstRun == true)
    {
        mPreviousData = mInputData;
        mbIsFirstRun = false;
    }
    mFilteredData = mAlpha * mInputData + (1 - mAlpha) * mPreviousData;
    mPreviousData = mFilteredData;
}


CanineFilter::MAF::MAF(const int &size)
        : mSize(size)
{
    mData = Eigen::VectorXd::Zero(mSize);
}

double CanineFilter::MAF::GetFilteredVar(const double &data)
{
    mInputValue = data;
    this->doFiltering();
    return mFilteredValue;
}

void CanineFilter::MAF::doFiltering()
{
    double sum;
    sum = 0;

    for(int i=0; i<(mSize-1); i++)
    {
        mData[i] = mData[i+1];
    }
    mData[mSize-1] = mInputValue;
    for(int i=0; i<mSize; i++)
    {
        sum += mData[i];
    }
    mFilteredValue = sum/mSize;
}

CanineFilter::Vec3MAF::Vec3MAF(const int& sizeX,const int& sizeY, const int& sizeZ)
    : mbIsFirstRun(true)
    , mSizeX(sizeX)
    , mSizeY(sizeY)
    , mSizeZ(sizeZ)
{
    mData[0] = Eigen::VectorXd::Zero(mSizeX);
    mData[1] = Eigen::VectorXd::Zero(mSizeY);
    mData[2] = Eigen::VectorXd::Zero(mSizeZ);
}

Vec3<double> CanineFilter::Vec3MAF::GetFilteredVar(const Vec3<double> &data)
{
    mInputValue = data;
    this->doFiltering();

//    std::cout << mInputValue[0] << std::endl;

    return mFilteredValue;
}

void CanineFilter::Vec3MAF::doFiltering()
{
    Vec3<double> sum ;
    sum << 0.0, 0.0, 0.0;

    /// update x
    for(int i=0; i<(mSizeX-1); i++)
    {
        mData[0][i] = mData[0][i+1];
    }
    /// update y
    for(int i=0; i<(mSizeY-1); i++)
    {
        mData[1][i] = mData[1][i+1];
    }
    /// update z
    for(int i=0; i<(mSizeZ-1); i++)
    {
        mData[2][i] = mData[2][i+1];
    }

    mData[0][mSizeX-1] = mInputValue[0];
    mData[1][mSizeY-1] = mInputValue[1];
    mData[2][mSizeZ-1] = mInputValue[2];

    for(int i=0; i<mSizeX; i++)
    {
        sum[0] += mData[0][i];
    }

    for(int i=0; i<mSizeY; i++)
    {
        sum[1] += mData[1][i];
    }

    for(int i=0; i<mSizeZ; i++)
    {
        sum[2] += mData[2][i];
    }

    mFilteredValue[0] = sum[0]/mSizeX;
    mFilteredValue[1] = sum[1]/mSizeY;
    mFilteredValue[2] = sum[2]/mSizeZ;
}