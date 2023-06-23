//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTVISUALIZATION_H
#define RAISIM_ROBOTVISUALIZATION_H

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

class RobotVisualization {
public:
    RobotVisualization(raisim::World* world,
                       raisim::ArticulatedSystem* robot,
                       raisim::RaisimServer* server);
    ~RobotVisualization();
    void VisualFunction();
    void UpdateFootStep();

private:
    void openRaisimServer();
    void updateVisual();

private:
    raisim::RaisimServer* mServer;
    raisim::ArticulatedSystem* mRobot;
    raisim::World* mWorld;
    raisim::Visuals* mDesiredFootPosBox[4];
    raisim::Visuals* mGlobalFootPosBox[4];
    raisim::Visuals* mLocalFootPosBox[4];
};


#endif //RAISIM_ROBOTVISUALIZATION_H
