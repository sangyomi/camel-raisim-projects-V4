//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULMAIN_HPP
#define RAISIM_SIMULMAIN_HPP

#include <iostream>
#include <QApplication>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <camel-tools/ThreadGenerator.hpp>

#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/JoystickInfo.hpp>

#include <canine_simulation/SimulStateEstimator.hpp>
#include <canine_simulation/SimulControlPanel.hpp>
#include <canine_simulation/SimulControlPanelHigh.hpp>
#include <canine_simulation/SimulVisualizer.hpp>
#include <canine_simulation/SimulCommand.hpp>
#include <canine_simulation/SimulCommunication.hpp>

void StartSimulation();
void clearSharedMemory();
void* NRTCommandThread(void* arg);
void* RTControllerThreadHigh(void *arg);
void* RTControllerThreadLow(void *arg);

#endif //RAISIM_SIMULMAIN_HPP
