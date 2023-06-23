#include <canine_util/CanMotorRB.hpp>
#include "camel-tools/ThreadGenerator.hpp"

extern pSHM sharedMemory;

CanMotorRB::CanMotorRB(std::string canName)
    : mCanName(canName)
{
    enc2rad = 2.0 * 3.141592 / 65535;
    mSock = 0;
    mGearRatio = 9;
    torque2int[RBHR_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = 29.9043;
    torque2int[RBHP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = 29.9043;
    torque2int[RBKP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = 24.0385;

    mMotorId[RBHR_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = MOTOR_RBHR_ID;
    mMotorId[RBHP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = MOTOR_RBHP_ID;
    mMotorId[RBKP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = MOTOR_RBKP_ID;

    mAxis[RBHR_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = -1.0;
    mAxis[RBHP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = -1.0;
    mAxis[RBKP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = -1.0;

    mAngularPositionOffset[RBHR_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = RBHR_POS_OFFSET;
    mAngularPositionOffset[RBHP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = RBHP_POS_OFFSET;
    mAngularPositionOffset[RBKP_IDX - RB_IDX * MOTOR_NUM_PER_CAN] = RBKP_POS_OFFSET;

    for (int index = 0; index < MOTOR_NUM_PER_CAN; index++)
    {
        mEncoder[index] = 0;
        mEncoderMultiturnNum[index] = 0;
        mEncoderTemp[index] = 35000;
        mEncoderPast[index] = 35000;
        mEncoderRaw[index] = 0;
        mEncoderOffset[index] = 0;
        mMotorTemperature[index] = 0;
        mMotorErrorCode[index] = 0;
        mAngularPosition[index] = 0;
        mAngularVelocity[index] = 0;
        mCurrentTorque[index] = 0;
        mMotorVoltage[index] = 0;
    }
}

void CanMotorRB::CanFunction()
{
    switch (sharedMemory->canRBState)
    {
    case CAN_NO_ACT:
    {
        usleep(10);
        break;
    }
    case CAN_MOTOR_ON:
    {
        turnOnMotor();
        sharedMemory->canRBState = CAN_NO_ACT;
        readEncoder();
        sharedMemory->motorRBState = true;
        break;
    }
    case CAN_INIT:
    {
        canInit();
        sharedMemory->canRBState = CAN_READ_ERROR;
        break;
    }
    case CAN_MOTOR_OFF:
    {
        turnOffMotor();
        sharedMemory->canRBState = CAN_NO_ACT;
        break;
    }
    case CAN_SET_TORQUE:
    {
        setTorque();
        break;
    }
    case CAN_READ_ERROR:
    {
        readMotorErrorStatus();
        break;
    }
    default:
        break;
    }
}

void CanMotorRB::canInit()
{
    std::string command3 =
        "sudo ip link set " + mCanName + " up type can bitrate 1000000";
    const char* c3 = command3.c_str();
    system(c3);
    mSock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mSock == -1)
    {
        perror("Fail to create can socket for ");
        std::cout << mCanName << std::endl;
        return;
    }
    std::cout << "Success to create can socket for " << mCanName << std::endl;

    struct ifreq ifr;
    const char* canName = mCanName.c_str();
    strcpy(ifr.ifr_name, canName);
    int ret = ioctl(mSock, SIOCGIFINDEX, &ifr);
    if (ret == -1)
    {
        perror("Fail to get can interface index -");
        return;
    }
    std::cout << "Success to get can interface index: " << ifr.ifr_ifindex << std::endl;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(mSock, (struct sockaddr*)&addr, sizeof(addr));
    if (ret == -1)
    {
        perror("Fail to bind can socket -");
        return;
    }
    std::cout << "Success to bind can socket" << std::endl;
    sharedMemory->canRBStatus = true;
}

void CanMotorRB::canSend(int motorIndex, const u_int8_t* data)
{
    u_int32_t tempid = mMotorId[motorIndex] & 0x1fffffff;
    mFrame.can_id = tempid;
    memcpy(mFrame.data, data, sizeof(data));
    mFrame.can_dlc = sizeof(data);
    int tx_bytes = write(mSock, &mFrame, sizeof(mFrame));
    if (tx_bytes == -1)
    {
        perror("Fail to transmit can");
        return;
    }
}

void CanMotorRB::canRead()
{
    read(mSock, &mFrame, sizeof(mFrame));
}

void CanMotorRB::readEncoder()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        u_int8_t data[8] = { 0X90, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
        canSend(motorIndex, data);
        canRead();

        mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
        mEncoderTemp[motorIndex] = mFrame.data[2] + mFrame.data[3] * 256;
        mEncoderRaw[motorIndex] = mFrame.data[4] + mFrame.data[5] * 256;
        mEncoderOffset[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000))
        {
            mEncoderMultiturnNum[motorIndex] += 1;
        }
        else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000))
        {
            mEncoderMultiturnNum[motorIndex] -= 1;
        }
        mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
        mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;
    }
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        sharedMemory->motorPosition[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mAxis[motorIndex] * (mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex]);
    }
}

void CanMotorRB::readMotorErrorStatus()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        u_int8_t data[8] = { 0X9a, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
        canSend(motorIndex, data);
        canRead();
        mMotorTemperature[motorIndex] = mFrame.data[1];
        mMotorVoltage[motorIndex] = (mFrame.data[3] + mFrame.data[4] * 256) * 0.1;
        mMotorErrorCode[motorIndex] = mFrame.data[7];
        sharedMemory->motorTemp[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mMotorTemperature[motorIndex];
        sharedMemory->motorVoltage[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mMotorVoltage[motorIndex];
        sharedMemory->motorErrorStatus[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mMotorErrorCode[motorIndex];
    }
}

void CanMotorRB::turnOffMotor()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        u_int8_t data[8] = { 0x80, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
        canSend(motorIndex, data);
        canRead();
    }
    sharedMemory->motorStatus = false;
}

void CanMotorRB::stopMotor()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        u_int8_t data[8] = { 0x81, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
        canSend(motorIndex, data);
        canRead();
    }
}

void CanMotorRB::turnOnMotor()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        u_int8_t data[8] = { 0x88, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
        canSend(motorIndex, data);
        canRead();
    }
    sleep(5);
    sharedMemory->motorStatus = true;
}

void CanMotorRB::setTorque()
{
    double desiredTorque[MOTOR_NUM_PER_CAN];
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        desiredTorque[motorIndex] = sharedMemory->motorDesiredTorque[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN];
//        desiredTorque[motorIndex] = 0.0;
    }

    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        u_int16_t desiredCurrent = round(mAxis[motorIndex] * torque2int[motorIndex] * desiredTorque[motorIndex]);
        u_int8_t currentLowerData;
        u_int8_t currentUpperData;

        currentLowerData = desiredCurrent % 256;
        desiredCurrent = desiredCurrent / 256;
        currentUpperData = desiredCurrent % 256;
        u_int8_t data[8] = { 0Xa1, 0X00, 0X00, 0X00, currentLowerData, currentUpperData, 0X00, 0X00 };

        canSend(motorIndex, data);
        canRead();

        int16_t currentTorque;
        int16_t angularVelocity;

        mMotorTemperature[motorIndex] = mFrame.data[1];
        currentTorque = mFrame.data[2] + mFrame.data[3] * 256;
        mCurrentTorque[motorIndex] = currentTorque / torque2int[motorIndex];
        angularVelocity = mFrame.data[4] + mFrame.data[5] * 256;
        mAngularVelocity[motorIndex] = angularVelocity * D2R / mGearRatio;

        mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
        mEncoderTemp[motorIndex] = mFrame.data[6] + mFrame.data[7] * 256;
        if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000))
        {
            mEncoderMultiturnNum[motorIndex] += 1;
        }
        else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000))
        {
            mEncoderMultiturnNum[motorIndex] -= 1;
        }
        mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
        mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;
    }

    for (int motorIndex = 0; motorIndex < MOTOR_NUM_PER_CAN; motorIndex++)
    {
        sharedMemory->motorTemp[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mMotorTemperature[motorIndex];
        sharedMemory->motorTorque[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mAxis[motorIndex] * mCurrentTorque[motorIndex];
        sharedMemory->motorVelocity[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mAxis[motorIndex] * mAngularVelocity[motorIndex];
        sharedMemory->motorPosition[motorIndex + RB_IDX * MOTOR_NUM_PER_CAN] = mAxis[motorIndex] * (mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex]);
    }
}