//
// Created by hs on 22. 11. 1.
//

#ifndef RAISIM_FILTER_HPP
#define RAISIM_FILTER_HPP

#include <iostream>
#include <canine_util/EigenTypes.hpp>

namespace CanineFilter
{
    class Vec3LPF{
    public:
        Vec3LPF(double DT, double cutoffFreq);
        Vec3<double> GetFilteredVar(const Vec3<double>& data);
    private:
        void doFiltering();
    private:
        bool mbIsFirstRun;
        double mDT;
        double mCutoffFreq;
        double mAlpha;
        Vec3<double> mInputData;
        Vec3<double> mPreviousData;
        Vec3<double> mFilteredData;
    };

    class LPF{
    public:
        LPF(double dt, double fc);
        double GetFilteredVar(double const data);
    private:
        void doFiltering();
    private:
        bool mbIsFirstRun;
        double mDT;
        double mCutoffFreq;
        double mAlpha;
        double mInputData;
        double mPreviousData;
        double mFilteredData;
    };

    class MAF{
    public:
        MAF(const int& size);
        double GetFilteredVar(const double& data);
    private:
        void doFiltering();
    private:
        const int mSize;
        Eigen::VectorXd mData;
        double mInputValue;
        double mFilteredValue;
    };

    class Vec3MAF{
    public:
        Vec3MAF(const int& sizeX,const int& sizeY, const int& sizeZ);
        Vec3<double> GetFilteredVar(const Vec3<double>& data);
    private:
        void doFiltering();
    private:
        bool mbIsFirstRun;
        const int mSizeX;
        const int mSizeY;
        const int mSizeZ;
        Eigen::VectorXd mData[3];
        Vec3<double> mInputValue;
        Vec3<double> mFilteredValue;
    };
}



#endif //RAISIM_FILTER_HPP
