//
// Created by cha on 22. 10. 5.
//

#include "../include/LordImu3DmGx5Ahrs.hpp"

static constexpr uint32_t Hash(const char* c)
{
    return *c ? static_cast<uint32_t>(*c) + 33 * Hash(c + 1) : 5381;
};

LordImu3DmGx5Ahrs::LordImu3DmGx5Ahrs(mscl::InertialNode* node)
{
    mNode = node;

    mAcceleration[0] = 0;
    mAcceleration[1] = 0;
    mAcceleration[2] = 0;

    mAngularVelocity[0] = 0;
    mAngularVelocity[1] = 0;
    mAngularVelocity[2] = 0;

    mEulerAngle[0] = 0;
    mEulerAngle[1] = 0;
    mEulerAngle[2] = 0;

    mLinearAcceleration[0] = 0;
    mLinearAcceleration[1] = 0;
    mLinearAcceleration[2] = 0;

    mQuaternion[0] = 0;
    mQuaternion[1] = 0;
    mQuaternion[2] = 0;
    mQuaternion[3] = 0;

    mStabilizedAcceleration[0] = 0;
    mStabilizedAcceleration[1] = 0;
    mStabilizedAcceleration[2] = 0;



    std::cout << "Model Name : " << mNode->modelName() << std::endl;
    std::cout << "Model Number : " << mNode->modelNumber() << std::endl;
    std::cout << "Serial : " << mNode->serialNumber() << std::endl;
}

void LordImu3DmGx5Ahrs::SetConfig(int samplingHz)
{
    mscl::MipChannels ahrsImuChs;
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_STABILIZED_ACCEL_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl::SampleRate::Hertz(samplingHz)));
    ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES, mscl::SampleRate::Hertz(samplingHz)));
    mNode->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);

    mscl::MipChannels estFilterChs;
    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, mscl::SampleRate::Hertz(samplingHz)));
//    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl::SampleRate::Hertz(samplingHz)));
//    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION, mscl::SampleRate::Hertz(samplingHz)));
    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, mscl::SampleRate::Hertz(samplingHz)));
//    estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE, mscl::SampleRate::Hertz(samplingHz)));

    mNode->setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);
}

void LordImu3DmGx5Ahrs::ParseData()
{
    mscl::MipDataPackets packets = mNode->getDataPackets(250);

    for (mscl::MipDataPacket packet : packets)
    {
        mscl::MipDataPoints data = packet.data();
        mscl::MipDataPoint dataPoint;
        for (unsigned int itr = 0; itr < data.size(); itr++)
        {
            dataPoint = data[itr];
//            std::cout << dataPoint.channelName() << std::endl;
            uint32_t hash = Hash(dataPoint.channelName().c_str());
            switch (hash)
            {
            case Hash("roll"):
//                mEulerAngle[0] = dataPoint.as_double();
//                break;
            case Hash("pitch"):
//                mEulerAngle[1] = dataPoint.as_double();
                break;
            case Hash("yaw"):
//                mEulerAngle[2] = dataPoint.as_double();
                break;
            case Hash("estRoll"):
                mEulerAngle[0] = dataPoint.as_double();
                break;
            case Hash("estPitch"):
                mEulerAngle[1] = dataPoint.as_double();
                break;
            case Hash("estYaw"):
                mEulerAngle[2] = dataPoint.as_double();
                break;

            case Hash("estHeading"):
                mHeading = dataPoint.as_double();
                break;

            case Hash("estOrientQuaternion"):
                mOrientQuaternion = dataPoint.as_string();
                break;

            case Hash("estAngularRateX"):
//                mAngularVelocity[0] = dataPoint.as_double();
                break;
            case Hash("estAngularRateY"):
//                mAngularVelocity[1] = dataPoint.as_double();
                break;
            case Hash("estAngularRateZ"):
//                mAngularVelocity[2] = dataPoint.as_double();
                break;

            case Hash("estLinearAccelX"):
                mLinearAcceleration[0] = dataPoint.as_double();
                break;
            case Hash("estLinearAccelY"):
                mLinearAcceleration[1] = dataPoint.as_double();
                break;
            case Hash("estLinearAccelZ"):
                mLinearAcceleration[2] = dataPoint.as_double();
                break;

            case Hash("scaledAccelX"):
                mAcceleration[0] = dataPoint.as_double();
                break;
            case Hash("scaledAccelY"):
                mAcceleration[1] = dataPoint.as_double();
                break;
            case Hash("scaledAccelZ"):
                mAcceleration[2] = dataPoint.as_double();
                break;

                ///
            case Hash("scaledGyroX"):
                mAngularVelocity[0] = dataPoint.as_double();
                break;
            case Hash("scaledGyroY"):
                mAngularVelocity[1] = dataPoint.as_double();
                break;
            case Hash("scaledGyroZ"):
                mAngularVelocity[2] = dataPoint.as_double();
                break;

            case Hash("stabilizedAccelX"):
                mStabilizedAcceleration[0] = dataPoint.as_double();
                break;
            case Hash("stabilizedAccelY"):
                mStabilizedAcceleration[1] = dataPoint.as_double();
                break;
            case Hash("stabilizedAccelZ"):
                mStabilizedAcceleration[2] = dataPoint.as_double();
                break;

            default:
//                std::cout << "default" << std::endl;
                break;
            }
            if (!dataPoint.valid())
            {
//                std::cout << "[Invalid] ";
            }
        }
    }
}

double* LordImu3DmGx5Ahrs::GetEulerAngle()
{
    return mEulerAngle;
}

double* LordImu3DmGx5Ahrs::GetQuaternion()
{
    std::string temp[4];
    for (int i = 1, j = 0; i < mOrientQuaternion.size() - 1; i++)
    {
        if (mOrientQuaternion[i] != ',')
        {
            temp[j].push_back(mOrientQuaternion[i]);
        }
        else
        {
            j++;
        }
    }
    mQuaternion[0] = std::stod(temp[0]);
    mQuaternion[1] = std::stod(temp[1]);
    mQuaternion[2] = std::stod(temp[2]);
    mQuaternion[3] = std::stod(temp[3]);

    return mQuaternion;
}

double* LordImu3DmGx5Ahrs::GetAcceleration()
{
    return mAcceleration;
}

double* LordImu3DmGx5Ahrs::GetStabilizedAcceleration()
{
    return mStabilizedAcceleration;
}

double* LordImu3DmGx5Ahrs::GetAngularVelocity()
{
    return mAngularVelocity;
}

double* LordImu3DmGx5Ahrs::GetLinearAcceleration()
{
    return mLinearAcceleration;
}

double LordImu3DmGx5Ahrs::GetHeading()
{
    return mHeading;
}

void LordImu3DmGx5Ahrs::GetCurrentConfig(mscl::InertialNode node)
{
        //many other settings are available than shown below
        //reference the documentation for the full list of commands

        //if the node supports AHRS/IMU
        if(node.features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
        {
            //get a list of the AHRS/IMU channels currently active on the Node
            mscl::MipChannels ahrsImuActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU);

            std::cout << "AHRS/IMU Channels" << std::endl;
            std::cout << "-----------------" << std::endl;
            for(mscl::MipChannel ch : ahrsImuActiveChs)
            {
                std::cout << "Channel Field: " << std::hex << ch.channelField() << std::endl;
                std::cout << "Sample Rate: " << ch.sampleRate().prettyStr() << std::endl << std::endl;
            }
        }

        //if the node supports Estimation Filter
        if(node.features().supportsCategory(mscl::MipTypes::CLASS_ESTFILTER))
        {
            //get a list of the Estimation Filter channels currently active on the Node
            mscl::MipChannels estFilterActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER);

            std::cout << std::endl;
            std::cout << "Estimation Filter Channels" << std::endl;
            std::cout << "--------------------------" << std::endl;
            for(mscl::MipChannel ch : estFilterActiveChs)
            {
                std::cout << "Channel Field: " << std::hex << ch.channelField() << std::endl;
                std::cout << "Sample Rate: " << ch.sampleRate().prettyStr() << std::endl << std::endl;
            }
        }
}