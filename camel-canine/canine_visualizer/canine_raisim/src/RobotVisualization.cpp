//
// Created by camel on 22. 9. 21.
//

#include <canine_raisim/RobotVisualization.hpp>

extern pSHM sharedMemory;

RobotVisualization::RobotVisualization(raisim::World* world, raisim::ArticulatedSystem* robot, raisim::RaisimServer* server)
    : mWorld(world)
    , mRobot(robot)
    , mServer(server)
{
    mWorld->setGravity({0.0, 0.0, -9.81});
    mWorld->setTimeStep(VISUAL_dT);
    mWorld->addGround();
    mRobot->setName("Canine");
    mDesiredFootPosBox[0] = mServer->addVisualBox("LF_desired_foot", 0.02, 0.02, 0.01, 1, 0, 0);
    mDesiredFootPosBox[1] = mServer->addVisualBox("RF_desired_foot", 0.02, 0.02, 0.01, 0, 1, 0);
    mDesiredFootPosBox[2] = mServer->addVisualBox("LB_desired_foot", 0.02, 0.02, 0.01, 1, 0, 1);
    mDesiredFootPosBox[3] = mServer->addVisualBox("RB_desired_foot", 0.02, 0.02, 0.01, 0, 1, 1);

    mGlobalFootPosBox[0] = mServer->addVisualBox("LF_global_foot", 0.02, 0.02, 0.01, 0.5, 0, 0);
    mGlobalFootPosBox[1] = mServer->addVisualBox("RF_global_foot", 0.02, 0.02, 0.01, 0, 0.5, 0);
    mGlobalFootPosBox[2] = mServer->addVisualBox("LB_global_foot", 0.02, 0.02, 0.01, 0.5, 0, 0.5);
    mGlobalFootPosBox[3] = mServer->addVisualBox("RB_global_foot", 0.02, 0.02, 0.01, 0, 0.5, 0.5);

    mLocalFootPosBox[0] = mServer->addVisualBox("LF_local_foot", 0.02, 0.02, 0.01, 0.5, 0, 0);
    mLocalFootPosBox[1] = mServer->addVisualBox("RF_local_foot", 0.02, 0.02, 0.01, 0, 0.5, 0);
    mLocalFootPosBox[2] = mServer->addVisualBox("LB_local_foot", 0.02, 0.02, 0.01, 0.5, 0, 0.5);
    mLocalFootPosBox[3] = mServer->addVisualBox("RB_local_foot", 0.02, 0.02, 0.01, 0, 0.5, 0.5);
}

RobotVisualization::~RobotVisualization()
{
    mServer->killServer();
}

void RobotVisualization::VisualFunction()
{
    switch(sharedMemory->visualState)
    {
        case STATE_VISUAL_STOP:
        {
            break;
        }
        case STATE_OPEN_RAISIM:
        {
//            openRaisimServer();
            sharedMemory->visualState = STATE_UPDATE_VISUAL;
            break;
        }
        case STATE_UPDATE_VISUAL:
        {
//            UpdateFootStep();
//            updateVisual();
            break;
        }
        default:
            break;
    }
}

void RobotVisualization::UpdateFootStep()
{
    for (int i = 0; i < 4; i++)
    {
        mDesiredFootPosBox[i]->setPosition(sharedMemory->desiredFootPosition[i][0],sharedMemory->desiredFootPosition[i][1],sharedMemory->desiredFootPosition[i][2]);
        mGlobalFootPosBox[i]->setPosition(sharedMemory->globalFootPosition[i][0],sharedMemory->globalFootPosition[i][1],sharedMemory->globalFootPosition[i][2]);
        mLocalFootPosBox[i]->setPosition(sharedMemory->bodyFootPosition[i][0],sharedMemory->bodyFootPosition[i][1],sharedMemory->bodyFootPosition[i][2]+0.5);
    }
}

void RobotVisualization::openRaisimServer()
{
    mServer->launchServer(8080);
    sleep(1);
}

void RobotVisualization::updateVisual()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();

    // base_x,y,z
    initialJointPosition[0] = sharedMemory->basePosition[0];
    initialJointPosition[1] = sharedMemory->basePosition[1];
    initialJointPosition[2] = sharedMemory->basePosition[2];

    // base_rotation [quaternion]
    initialJointPosition[3] = sharedMemory->baseQuartPosition[0];
    initialJointPosition[4] = sharedMemory->baseQuartPosition[1];
    initialJointPosition[5] = sharedMemory->baseQuartPosition[2];
    initialJointPosition[6] = sharedMemory->baseQuartPosition[3];

    // LF_hip,thigh,calf
    initialJointPosition[7] = sharedMemory->motorPosition[LFHR_IDX];
    initialJointPosition[8] = sharedMemory->motorPosition[LFHP_IDX];
    initialJointPosition[9] = sharedMemory->motorPosition[LFKP_IDX];

    // RF_hip,thigh,calf
    initialJointPosition[10] = sharedMemory->motorPosition[RFHR_IDX];
    initialJointPosition[11] = sharedMemory->motorPosition[RFHP_IDX];
    initialJointPosition[12] = sharedMemory->motorPosition[RFKP_IDX];

    // LB_hip,thigh,calf
    initialJointPosition[13] = sharedMemory->motorPosition[LBHR_IDX];
    initialJointPosition[14] = sharedMemory->motorPosition[LBHP_IDX];
    initialJointPosition[15] = sharedMemory->motorPosition[LBKP_IDX];

    // RB_hip,thigh,calf
    initialJointPosition[16] = sharedMemory->motorPosition[RBHR_IDX];
    initialJointPosition[17] = sharedMemory->motorPosition[RBHP_IDX];
    initialJointPosition[18] = sharedMemory->motorPosition[RBKP_IDX];
    mRobot->setGeneralizedCoordinate(initialJointPosition);
}
