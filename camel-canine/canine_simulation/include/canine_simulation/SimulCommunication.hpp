//
// Created by sangjun on 23. 5. 10.
//

#ifndef QTTCP_TCPCOMMUNICATION_HPP
#define QTTCP_TCPCOMMUNICATION_HPP

#include <QTcpServer>
#include <QTcpSocket>
#include <QDataStream>
#include <QApplication>
#include <QtCore>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/JoystickInfo.hpp>
#include <camel-tools/ThreadGenerator.hpp>

void deserializeJoystickInfo(QDataStream &stream);
void serializeSharedMemoryInfo (QDataStream &stream);
void* receiveData(void *arg);
void* sendData(void* arg);
void StartCommunication();


#endif //QTTCP_TCPCOMMUNICATION_HPP
