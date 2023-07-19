//
// Created by sangjun on 23. 5. 10.
//

#include <canine_simulation/SimulCommunication.hpp>

extern pAXIS joystickAxis;
extern pBUTTON joystickButton;
extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;
extern pCUSTOM_DATA sharedCustom;

pthread_t QtServer;
pthread_t QtClient;

void deserializeJoystickInfo(QDataStream &stream) {
    stream >> joystickAxis->LeftStickX;
    stream >> joystickAxis->LeftStickY;
    stream >> joystickAxis->LeftTrigger;
    stream >> joystickAxis->RightStickX;
    stream >> joystickAxis->RightStickY;
    stream >> joystickAxis->RightTrigger;
    stream >> joystickAxis->DpadX;
    stream >> joystickAxis->DpadY;

    stream >> joystickButton->FaceButtonA;
    stream >> joystickButton->FaceButtonB;
    stream >> joystickButton->FaceButtonX;
    stream >> joystickButton->FaceButtonY;
    stream >> joystickButton->LeftBumper;
    stream >> joystickButton->RightBumper;
    stream >> joystickButton->Back;
    stream >> joystickButton->Start;
    stream >> joystickButton->Guide;
    stream >> joystickButton->LeftStick;
    stream >> joystickButton->RightStick;
}

void serializeSharedMemoryInfo (QDataStream &stream)
{
    // UI_COMMAND
    stream << sharedCommand->userCommand;
    stream << sharedCommand->gaitCommand;
    QString string;
    stream << string;
    QByteArray byteArray = string.toUtf8();
    Q_ASSERT(byteArray.size() <= MAX_COMMAND_DATA);
    qstrncpy(sharedCommand->userParamChar, byteArray.constData(), MAX_COMMAND_DATA);
    stream.writeRawData(reinterpret_cast<char*>(sharedCommand->userParamInt), MAX_COMMAND_DATA * sizeof(int));
    stream.writeRawData(reinterpret_cast<char*>(sharedCommand->userParamDouble), MAX_COMMAND_DATA * sizeof(double));

    // SHM
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->gaitTable), MPC_HORIZON*4 * sizeof(int));
    stream << sharedMemory->gaitState;
    stream << sharedMemory->gaitPeriod;
    stream << sharedMemory->swingPeriod;
    stream << sharedMemory->standPeriod;
    stream << sharedMemory->gaitChangeFlag;

    stream << sharedMemory->throwFlag;

    stream << sharedMemory->isNan;
    stream << sharedMemory->isRamp;
    stream << sharedMemory->bIsEndHome;
    stream << sharedMemory->newCommand;

    stream << sharedMemory->canLFStatus;
    stream << sharedMemory->canRFStatus;
    stream << sharedMemory->canLBStatus;
    stream << sharedMemory->canRBStatus;

    stream << sharedMemory->motorStatus;
    stream << sharedMemory->motorLFState;
    stream << sharedMemory->motorRFState;
    stream << sharedMemory->motorLBState;
    stream << sharedMemory->motorRBState;

    stream << sharedMemory->FSMState;
    stream << sharedMemory->LowControlState;
    stream << sharedMemory->HighControlState;
    stream << sharedMemory->visualState;

    stream << sharedMemory->canLFState;
    stream << sharedMemory->canRFState;
    stream << sharedMemory->canLBState;
    stream << sharedMemory->canRBState;

    stream << sharedMemory->localTime;

    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorErrorStatus), MOTOR_NUM * sizeof(int));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorTemp), MOTOR_NUM * sizeof(int));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorVoltage), MOTOR_NUM * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorPosition), MOTOR_NUM * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorTorque), MOTOR_NUM * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorDesiredPosition), MOTOR_NUM * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorDesiredVelocity), MOTOR_NUM * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorDesiredTorque), MOTOR_NUM * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->motorPrevDesiredPosition), MOTOR_NUM * sizeof(double));

    stream << sharedMemory->basePosition(0) << sharedMemory->basePosition(1) << sharedMemory->basePosition(2);
    stream << sharedMemory->baseVelocity(0) << sharedMemory->baseVelocity(1) << sharedMemory->baseVelocity(2);
    stream << sharedMemory->baseQuartPosition(0) << sharedMemory->baseQuartPosition(1) << sharedMemory->baseQuartPosition(2) << sharedMemory->baseQuartPosition(3);
    stream << sharedMemory->baseDesiredPosition(0) << sharedMemory->baseDesiredPosition(1) << sharedMemory->baseDesiredPosition(2);
    stream << sharedMemory->baseDesiredVelocity(0) << sharedMemory->baseDesiredVelocity(1) << sharedMemory->baseDesiredVelocity(2);
    stream << sharedMemory->baseDesiredQuartPosition(0) << sharedMemory->baseDesiredQuartPosition(1) << sharedMemory->baseDesiredQuartPosition(2) << sharedMemory->baseDesiredQuartPosition(3);
    stream << sharedMemory->baseDesiredEulerPosition(0) << sharedMemory->baseDesiredEulerPosition(1) << sharedMemory->baseDesiredEulerPosition(2);
    stream << sharedMemory->baseDesiredEulerVelocity(0) << sharedMemory->baseDesiredEulerVelocity(1) << sharedMemory->baseDesiredEulerVelocity(2);
    stream << sharedMemory->baseLocalDesiredVelocity(0) << sharedMemory->baseLocalDesiredVelocity(1) << sharedMemory->baseLocalDesiredVelocity(2);
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->baseEulerPosition), 3 * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->baseEulerVelocity), 3 * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->baseAcceleration), 3 * sizeof(double));

    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->pdTorque[i](0) << sharedMemory->pdTorque[i](1) << sharedMemory->pdTorque[i](2);
    }
    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->mpcTorque[i](0) << sharedMemory->mpcTorque[i](1) << sharedMemory->mpcTorque[i](2);
    }
    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->bodyFootPosition[i](0) << sharedMemory->bodyFootPosition[i](1) << sharedMemory->bodyFootPosition[i](2);
    }
    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->globalFootPosition[i](0) << sharedMemory->globalFootPosition[i](1) << sharedMemory->globalFootPosition[i](2);
    }
    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->desiredFootPosition[i](0) << sharedMemory->desiredFootPosition[i](1) << sharedMemory->desiredFootPosition[i](2);
    }
    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->visualPosition[i](0) << sharedMemory->visualPosition[i](1) << sharedMemory->visualPosition[i](2);
    }

    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->contactState), 4 * sizeof(bool));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->tempRes), 4 * sizeof(double));
    stream << sharedMemory->simulContactForceFL;
    stream << sharedMemory->simulContactForceFR;
    stream << sharedMemory->simulContactForceHL;
    stream << sharedMemory->simulContactForceHR;

    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->testBasePos), 3 * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->testBaseVel), 3 * sizeof(double));

    for (int i = 0 ; i < 4 ; ++i){
        stream << sharedMemory->solvedGRF[i](0) << sharedMemory->solvedGRF[i](1) << sharedMemory->solvedGRF[i](2);
    }

    // CUSTOM_DATA
    stream.writeRawData(reinterpret_cast<char*>(sharedCustom->customVariableDouble), MAX_CUSTOM_DATA * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedCustom->customVariableInt), MAX_CUSTOM_DATA * sizeof(int));
}

void* receiveData(void *arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;

    const QHostAddress serverAddress(QHostAddress::Any);
    const quint16 serverPort = 12345;

    QTcpServer server;
    server.listen(serverAddress, serverPort);

    while (server.isListening()) {
        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
        if (server.waitForNewConnection()) {
            QTcpSocket *socket = server.nextPendingConnection();

            if (socket) {
                QDataStream in(socket);
                in.setVersion(QDataStream::Qt_5_0);

                qint64 blockSize = 0;
                if (socket->bytesAvailable() < sizeof(qint64)) {
                    socket->waitForReadyRead();
                }
                in >> blockSize;

                QByteArray data;
                while (socket->bytesAvailable() < blockSize) {
                    socket->waitForReadyRead();
                }
                data = socket->read(blockSize);

                QDataStream dataStream(&data, QIODevice::ReadOnly);

                deserializeJoystickInfo(dataStream);

                if (dataStream.status() != QDataStream::Ok) {
                    qWarning() << "Error while reading data: " << dataStream.status();
                }
                else
                {
//                    qDebug() << "deserializeJoystickInfo is done";
//                    qDebug() << "joystickButton->FaceButtonA: " << joystickButton->FaceButtonA;;
//                    qDebug() << "joystickAxis->LeftTrigger: " << joystickAxis->LeftTrigger;
                }

                QByteArray requestData;
                QDataStream out(&requestData, QIODevice::WriteOnly);
                out << (qint64)0;
                out.device()->seek(0);
                out << (qint64)data.size();

                socket->write(requestData);
                socket->write(data);
                socket->flush();

                socket->disconnectFromHost();
                socket->deleteLater();
            }
        }
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
//        std::cout << "[COMMUNICATION] receive data RT Thread time : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        usleep(100);
    }
}

void* sendData(void* arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;

//    const QHostAddress serverAddress("192.168.0.159"); //wifi
    const QHostAddress serverAddress("10.42.0.129"); // hotspot
    const quint16 serverPort = 34567;

    while(true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
        QTcpSocket* socket = new QTcpSocket();
        socket->connectToHost(serverAddress, serverPort);

        if (socket->waitForConnected())
        {
            QByteArray byteArray;
            QDataStream stream(&byteArray, QIODevice::WriteOnly);

            serializeSharedMemoryInfo(stream);

            QByteArray requestData;
            QDataStream out(&requestData, QIODevice::WriteOnly);
            out << (qint64)0;
            out.device()->seek(0);
            out << (qint64)byteArray.size();

            socket->write(requestData);
            socket->write(byteArray);
            socket->flush();

            socket->disconnectFromHost();
            socket->deleteLater();

//            if (socket->waitForReadyRead()) {
//                QDataStream in(socket);
//                in.setVersion(QDataStream::Qt_5_0);
//
//                qint64 blockSize = 0;
//
//          if (socket->bytesAvailable() < sizeof(qint64)) {
//                    socket->waitForReadyRead();
//                }
//                in >> blockSize;
//
//                QByteArray data;
//                while (socket->bytesAvailable() < blockSize) {
//                    socket->waitForReadyRead();
//                }
//                data = socket->read(blockSize);
//
//                QDataStream dataStream(&data, QIODevice::ReadOnly);
//
//                if (dataStream.status() != QDataStream::Ok) {
//                    qWarning() << "Error while reading data: " << dataStream.status();
//                }
//                else
//                {
////                    qDebug() << "serializeSharedMemoryInfo is done";
////                    qDebug() << "sharedCommand->userCommand: " << sharedCommand->userCommand;
////                    qDebug() << "sharedCommand->userParamChar: " << sharedCommand->userParamChar;
//                }
//
//                socket->disconnectFromHost();
//                socket->deleteLater();
//            }
        }
        else {
            qWarning() << "Could not connect to server: " << socket->errorString();
        }
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        std::cout << "[COMMUNICATION] send data RT Thread time : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        usleep(20000);
    }

}

void StartCommunication()
{
    int thread_id_rt2 = generate_rt_thread(QtClient, sendData, "client_thread", 6, 0,NULL);
    int thread_id_rt3 = generate_rt_thread(QtServer, receiveData, "server_thread", 4, 0, NULL);
}