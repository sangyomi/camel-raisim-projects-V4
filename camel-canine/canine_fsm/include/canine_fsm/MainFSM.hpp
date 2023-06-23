//
// Created by hs on 22. 10. 5.
//

#ifndef RAISIM_MAINFSM_HPP
#define RAISIM_MAINFSM_HPP

#include <iostream>
#include <QApplication>

#include <camel-tools/ThreadGenerator.hpp>
#include <camel-tools/sensor.hpp>

#include <ControlMain/LowControlMain.hpp>
#include <ControlMain/HighControlMain.hpp>

#include <canine_util/CanMotorLF.hpp>
#include <canine_util/CanMotorRF.hpp>
#include <canine_util/CanMotorLB.hpp>
#include <canine_util/CanMotorRB.hpp>
#include <canine_util/Command.hpp>
#include <canine_util/DataAnalysis.hpp>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/StateEstimator.hpp>
#include <canine_util/RobotMath.hpp>

#include <canine_raisim//RobotVisualization.hpp>

#include <canine_simulation/SimulCommunication.hpp>

void StartFSM();
void clearSharedMemory();
void* RTControllerThreadHigh(void *arg);
void* RTControllerThreadLow(void *arg);
void* RTImuThread(void* arg);
void* NRTCommandThread(void *arg);
void* NRTVisualThread(void *arg);
void* NRTCANLF(void *arg);
void* NRTCANRF(void *arg);
void* NRTCANLB(void *arg);
void* NRTCANRB(void *arg);


#endif //RAISIM_MAINFSM_HPP
