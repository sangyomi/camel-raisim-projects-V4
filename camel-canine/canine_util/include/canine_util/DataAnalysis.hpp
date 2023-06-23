//
// Created by camel on 23. 1. 23.
//

#ifndef RAISIM_DATAANALYSIS_HPP
#define RAISIM_DATAANALYSIS_HPP

#include <fstream>
#include <iostream>
#include <ctime>
#include <experimental/filesystem>

#include "SharedMemory.hpp"

class DataAnalysis{
public:
    DataAnalysis();
    ~DataAnalysis();

    void MakeFile();
    void SaveRobotState();

private:
    std::string mHomeLocation;
    std::string mExtension;
    std::string mFileName;

    std::ofstream mStates;
    std::ofstream mNonLinear;
    std::ofstream mJoints[4];
};

#endif //RAISIM_DATAANALYSIS_HPP
