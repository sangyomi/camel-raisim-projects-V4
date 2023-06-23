//
// Created by camel on 22. 9. 13.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

#include "RobotDescription.hpp"
#include "EigenTypes.hpp"

#define CMD_dT              0.002
#define HIGH_CONTROL_dT     0.02
#define LOW_CONTROL_dT      0.002
#define CAN_dT              0.0025
#define VISUAL_dT           0.01
#define IMU_dT              0.005
#define ESTIMATOR_dT        0.002
#define MAX_COMMAND_DATA    10
#define MAX_CUSTOM_DATA     20
#define PI                  3.141592
#define R2D                 57.2957802
#define D2R                 0.0174533
#define MPC_HORIZON         5


typedef struct _UI_COMMAND_
{
    int userCommand;
    int gaitCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, * pUI_COMMAND;

typedef struct _SHM_
{
    //Gait
    int gaitTable[MPC_HORIZON*4];
    int gaitState;
    double gaitPeriod;
    double swingPeriod;
    double standPeriod;
    bool gaitChangeFlag;

    bool throwFlag;

    bool isNan;
    bool isRamp;
    bool bIsEndHome;
    bool newCommand;

    bool canLFStatus;
    bool canRFStatus;
    bool canLBStatus;
    bool canRBStatus;

    bool motorStatus;
    bool motorLFState;
    bool motorRFState;
    bool motorLBState;
    bool motorRBState;

    int FSMState;
    int LowControlState;
    int HighControlState;
    int visualState;

    int canLFState;
    int canRFState;
    int canLBState;
    int canRBState;

    double localTime;

    int motorErrorStatus[MOTOR_NUM];
    int motorTemp[MOTOR_NUM];
    double motorVoltage[MOTOR_NUM];
    double motorPosition[MOTOR_NUM];
    double motorVelocity[MOTOR_NUM];
    double motorTorque[MOTOR_NUM];
    double motorDesiredPosition[MOTOR_NUM];
    double motorDesiredVelocity[MOTOR_NUM];
    double motorDesiredTorque[MOTOR_NUM];
    double motorPrevDesiredPosition[MOTOR_NUM];

    Vec3<double> basePosition;
    Vec3<double> baseVelocity;
    Vec4<double> baseQuartPosition;
    Vec3<double> baseDesiredPosition;
    Vec3<double> baseDesiredVelocity;
    Vec4<double> baseDesiredQuartPosition;
    Vec3<double> baseDesiredEulerPosition;
    Vec3<double> baseDesiredEulerVelocity;
    Vec3<double> baseLocalDesiredVelocity;
    double baseEulerPosition[3];
    double baseEulerVelocity[3];
    double baseAcceleration[3];

    Vec3<double> pdTorque[4];
    Vec3<double> mpcTorque[4];
    Vec3<double> bodyFootPosition[4];
    Vec3<double> globalFootPosition[4];
    Vec3<double> desiredFootPosition[4];
    Vec3<double> visualPosition[4];

    /// contact detction
    bool contactState[4];
    double tempRes[4];
    double simulContactForceFL;
    double simulContactForceFR;
    double simulContactForceHL;
    double simulContactForceHR;
    ///
    double testBasePos[3];
    double testBaseVel[3];

    Vec3<double> solvedGRF[4];

} SHM, * pSHM;

typedef struct _CUSTOM_DATA_
{
    double customVariableDouble[MAX_CUSTOM_DATA];
    int customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, * pCUSTOM_DATA;



#endif //RAISIM_SHAREDMEMORY_H
