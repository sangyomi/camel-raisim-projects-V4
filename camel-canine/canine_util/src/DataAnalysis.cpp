#include <canine_util/DataAnalysis.hpp>

extern pSHM sharedMemory;

DataAnalysis::DataAnalysis()
        : mHomeLocation("/home/camel/CanineV1_MO/")
        , mExtension(".csv")
{
}

DataAnalysis::~DataAnalysis()
{
    std::cout << "[INFO] "<< mFileName << " is saved" << std::endl;
    mStates.close();
    for (int idx=0; idx<4; idx++)
    {
        mJoints[idx].close();
    }
}

void DataAnalysis::MakeFile()
{
    time_t timer;
    struct tm* t;
    timer = time(NULL);
    t = localtime(&timer);

    std::string currentTime = std::to_string(t->tm_year+1900) + "_"
                              + std::to_string(t->tm_mon+1) + "_"
                              + std::to_string(t->tm_mday) + "_"
                              + std::to_string(t->tm_hour) + "_"
                              + std::to_string(t->tm_min) + "_"
                              + std::to_string(t->tm_sec);

    mFileName = std::string("CanineV1_Residual_") + currentTime;

    mHomeLocation += mFileName + "/";
    std::experimental::filesystem::create_directory(mHomeLocation);

    mStates.open(mHomeLocation + mFileName + std::string("moOb") + mExtension);
    mStates << "t" << "\t"
            << "residual0" << "\t" << "residual1" << "\t" << "residual2" << "\t"
            << "residual3" << "\t" << "residual4" << "\t" << "residual5" << "\t"
            << "residual6" << "\t" << "residual7" << "\t" << "residual8" << "\t"
            << "residual9" << "\t" << "residual10" << "\t" << "residual11" << "\t"
            << "nonLinearFL" << "\t" << "nonLinearFR" << "\t" << "nonLinearRL" << "\t" << "nonLinearRR" << "\t"
            << "mResFL" << "\t" << "mResFR" << "\t" << "mResRL" << "\t" << "mResRR" << "\t"
            << "conFL" << "\t" << "conFR" << "\t" << "conRL" << "\t" << "conRR" << "\t"
            << "gaitFL" << "\t" << "gaitFR" << "\t" << "gaitRL" << "\t" << "gaitRR" << "\n";
}

void DataAnalysis::SaveRobotState()
{
    if (mStates.is_open())
    {
        mStates << sharedMemory->localTime << "\t"

                << sharedMemory->gaitTable[0] << "\t" << sharedMemory->gaitTable[1] << "\t" << sharedMemory->gaitTable[2] << "\t" << sharedMemory->gaitTable[3] << "\n";
    }
}
