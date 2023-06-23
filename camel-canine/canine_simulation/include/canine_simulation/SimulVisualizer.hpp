//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULVISUALIZER_HPP
#define RAISIM_SIMULVISUALIZER_HPP

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <canine_util/SharedMemory.hpp>
#include <canine_util/RobotDescription.hpp>

class SimulVisualizer {
public:
    SimulVisualizer(raisim::World* world,
                       raisim::ArticulatedSystem* robot,
                       raisim::RaisimServer* server);
    ~SimulVisualizer();
    void UpdateFootStep();

private:
    void initRobotPose();

private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;
    raisim::RaisimServer* mServer;
    raisim::Visuals* mBasePositionBox;
    raisim::Visuals* mDesiredFootPosBox[4];
    raisim::Visuals* mGlobalFootPosBox[4];
    raisim::Visuals* mLocalFootPosBox[4];
};

#endif //RAISIM_SIMULVISUALIZER_HPP
