// ----------------------------------------------------------------------------
// Copyright 2015 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
// This defines the communication between controller and ROS
// ----------------------------------------------------------------------------

#include <connector.h>
#include <iostream>

using namespace std;
using namespace ur_driver;
using namespace boost::asio::ip;

//=================================================================
// RobotState
//=================================================================
RobotState::RobotState()
{

}

RobotState::RobotState(const JointPosition& jointPosition, const JointVelocity& jointVelocity, const CartesianPosition& cartesianPosition)
{
    this->jointPosition = jointPosition;
    this->jointVelocity = jointVelocity;
    this->cartesianPosition = cartesianPosition;
}

JointPosition& RobotState::getJointPosition()
{
    return jointPosition;
}

void RobotState::setJointPosition(const JointPosition& jointPosition)
{
    this->jointPosition = jointPosition;
}

JointVelocity& RobotState::getJointVelocity()
{
    return jointVelocity;
}

void RobotState::setJointVelocity(const JointVelocity& jointVelocity)
{
    this->jointVelocity = jointVelocity;
}

CartesianPosition& RobotState::getCartesianPosition()
{
    return cartesianPosition;
}

void RobotState::setCartesianPosition(const CartesianPosition& cartesianPosition)
{
    this->cartesianPosition = cartesianPosition;
}

//=================================================================
// Connector
//=================================================================
Connector::Connector() :
    runConnectSocketThread(false),
    runReadSocketThread(false),
    runWriteSocketThread(false),
    socket(io),
    isRunning(false),
    host("localhost"),
    port(SECONDARY),
    isDummy(true),
    readFrequency(20),
    writeFrequency(20)
{

}

Connector::~Connector()
{
    disconnect();
}

void Connector::addCommand(Command* command)
{
    mutexCommandQueue.lock();

    if (commandQueue.size() > 10)
    {
        ROS_WARN_NAMED("connector", "command queue size: %i", (int)commandQueue.size());
    }

    commandQueue.push(command);

    mutexCommandQueue.unlock();
}

void Connector::connect(std::string host, int port, bool isDummy, double readFrequency, double writeFrequency)
{
    mutexStartStop.lock();

    if (!isRunning)
    {
        ROS_DEBUG_NAMED("connector", "connect to robot controller");

        this->host = host;
        this->port = port;
        this->isDummy = isDummy;
        this->readFrequency = readFrequency;
        this->writeFrequency = writeFrequency;

        //start dummy server
        if (isDummy)
        {
            dummy.start(port);
        }

        //clear command queue
        while(!commandQueue.empty())
        {
            delete commandQueue.front();
            commandQueue.pop();
        }

        //start connection thread
        runConnectSocketThread = true;
        connectSocketThread = boost::thread(boost::bind(&Connector::connectSocketWorker, this));

        isRunning = true;
    }

    mutexStartStop.unlock();
}

void Connector::disconnect()
{
    mutexStartStop.lock();

    if (isRunning)
    {
        ROS_DEBUG_NAMED("connector", "disconnect from robot controller");

        runConnectSocketThread = false;
        runReadSocketThread = false;
        runWriteSocketThread = false;

        try
        {
            socket.shutdown(tcp::socket::shutdown_both);
            socket.close();
        }
        catch (std::exception& e)
        {
        }

        connectSocketThread.join();
        readSocketThread.join();
        writeSocketThread.join();

        if (isDummy)
        {
            dummy.stop();
        }

        isRunning = false;
    }

    mutexStartStop.unlock();
}

void Connector::notifyListeners(RobotState& robotState)
{
    signalRobotState(robotState);
}

void Connector::connectSocketWorker()
{
    int retries = 0;
    while(runConnectSocketThread)
    {
        try
        {
            //wait 3 seconds if retry already failed
            if (retries > 0)
            {
                for (int i = 0; i < 30 && runConnectSocketThread; i++)
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                }
            }

            retries++;

            if (runConnectSocketThread)
            {
                //TODO support other ports
                if (/*port != InterfacePort::PRIMARY && */port != SECONDARY/* && port != InterfacePort::REALTIME*/)
                {
                    ROS_ERROR("port %i not supported", port);
                }
                else
                {
                    //resolve host
                    tcp::resolver::query query(host, boost::lexical_cast<std::string>(port));
                    tcp::resolver resolver(io);
                    tcp::resolver::iterator endpointIterator = resolver.resolve(query);

                    //connect to first resolved endpoint
                    socket.connect(*endpointIterator);

                    ROS_INFO_NAMED("connector", "connection established to %s:%i", host.c_str(), port);

                    //start read/write worker threads
                    runReadSocketThread = true;
                    runWriteSocketThread = true;
                    readSocketThread = boost::thread(boost::bind(&Connector::readSocketWorker, this));
                    writeSocketThread = boost::thread(boost::bind(&Connector::writeSocketWorker, this));

                    //reset retries (no exception was thrown, therefore a connection was established)
                    retries = 0;

                    //wait for read/write worker threads to finish
                    readSocketThread.join();
                    writeSocketThread.join();

                    ROS_INFO_NAMED("connector", "disconnected from %s:%i", host.c_str(), port);
                }
            }
        }
        catch (std::exception& e)
        {
            ROS_WARN_NAMED("connector", "connection to %s:%i failed: %s", host.c_str(), port, e.what());
        }
    }

    ROS_DEBUG_NAMED("connector", "exit connectSocketWorker thread");
}

void Connector::readSocketWorker()
{
    ros::Rate rate = ros::Rate(readFrequency);

    while(runReadSocketThread && socket.is_open())
    {
        try
        {
            boost::system::error_code error;

            //===========================
            // 1. read package size
            //===========================
            char dataPackageSize[4];
            size_t length = socket.read_some(boost::asio::buffer(dataPackageSize), error); //reads only 4 bytes of overall comming package size

            //connection closed cleanly by peer.
            if (error == boost::asio::error::eof)
            {
                socket.close();

                break;
            }
            //some other error
            else if (error)
            {
                throw boost::system::system_error(error);
            }

            //verify read byte length
            if (length != 4)
            {
                throw length_error("socket read: data package size: read byte length (" + boost::lexical_cast<string>(length) + ") differs from expected length (4)");
            }

            // Calculate packagesize
            uint32_t packageSize = (dataPackageSize[3] << 24) | (dataPackageSize[2] << 16) | (dataPackageSize[1] << 8) | dataPackageSize[0];
            packageSize = be32toh(packageSize) - 4;

            //print dataPackageContent stream in hex format
            ROS_DEBUG_NAMED("connector", "socket read: data package size (%i): %s", (int)length, hexString(dataPackageSize, length).c_str());

            //===========================
            // 2. read package content
            //===========================
            char dataPackageContent[1024];
            length = socket.read_some(boost::asio::buffer(dataPackageContent), error); // reads the rest of the package

            //connection closed cleanly by peer.
            if (error == boost::asio::error::eof)
            {
                break;
            }
            //some other error
            else if (error)
            {
                throw boost::system::system_error(error);
            }

            //verify read byte length
            if (length != packageSize) //throws if read package size does not match with received package size
            {
                throw length_error("socket read: data package content: read byte length (" + boost::lexical_cast<string>(length) + ") differs from expected length (" + boost::lexical_cast<string>(packageSize) + ")");
            }

            //print dataPackageContent stream in hex format, and only the content part (without the first 4byte package size info!)
            ROS_DEBUG_NAMED("connector", "socket read: data package content (%i): %s", (int)length, hexString(dataPackageContent, length).c_str());

            if (port == PRIMARY)
            {
                //TODO support port
                ROS_WARN("port %i not supported", port);
            }
            else if (port == SECONDARY)
            {
                ROS_WARN_COND(dataPackageContent[0] != 16, "Something went wrong: First packageType must always be 16 but was %i", dataPackageContent[0]);

                //process data package
                RobotState robotState;

                JointPosition jointPosition(6);
                JointVelocity jointVelocity(6);
                CartesianPosition cartesianPosition;

                int bytepointer = 1; // first byte of message was consumed as RobotMessageType

                while (bytepointer < packageSize)
                {
                    Packet_port30002::PacketHeader *packageHeader = (Packet_port30002::PacketHeader*)&dataPackageContent[bytepointer];
                    packageHeader->fixByteOrder();
                    //ROS_WARN("Type: %i, Length: %i", packageHeader->packageType, packageHeader->packageLength);
                    switch (packageHeader->packageType)
                    {
                        case 0:
                        {
                            Packet_port30002::RobotMode *robotmode = (Packet_port30002::RobotMode*)&dataPackageContent[bytepointer+sizeof(Packet_port30002::PacketHeader)];
                            ROS_DEBUG("RobotMode Package received");
                            robotmode->fixByteOrder();
                            robotState.isUrProgramRunning = robotmode->isProgramRunning;
                            robotState.isUrProgramPaused = robotmode->isProgramPaused;
                            break;
                        }
                        case 1:
                        {
                            for (int jointCnt=0; jointCnt<6; jointCnt++)
                            {
                                Packet_port30002::Joint *jointData =
                                        (Packet_port30002::Joint*)&dataPackageContent[bytepointer+sizeof(Packet_port30002::PacketHeader)+jointCnt*sizeof(Packet_port30002::Joint)];
                                jointData->fixByteOrder();
                                jointPosition[jointCnt] = jointData->q_act;
                                jointVelocity[jointCnt] = jointData->qd_act;
                            }
                            ROS_DEBUG("JointData Package received");
                        break;
                        }
                        case 2:
                        {
                            Packet_port30002::ToolData *toolData = (Packet_port30002::ToolData*)&dataPackageContent[bytepointer+sizeof(Packet_port30002::PacketHeader)];
                            ROS_DEBUG("ToolData Package received");
                            // Do the following before using it!
                            //toolData->fixByteOrder();
                        break;
                        }
                        case 3:
                        {
                            Packet_port30002::MasterboardData *masterBoardData = (Packet_port30002::MasterboardData*)&dataPackageContent[bytepointer+sizeof(Packet_port30002::PacketHeader)];
                            masterBoardData->fixByteOrder();
                            ROS_DEBUG("MasterboardData Package received");

                            // Read Input State 0 to 7 -> robotState.getIO(0..7)
                            for (int i=0; i<8; i++)
                            {
                                robotState.set_IO(i, masterBoardData->bit_to_bool(masterBoardData->DigitalnputBits, i));
                                ROS_INFO("Input State %d: %d", i, robotState.get_IO(i));
                            }

                            // Read Output State 0 to 7 -> robotState.getIO(8..15)
                            for (int i=0; i<8; i++)
                            {
                                robotState.set_IO(18+i, masterBoardData->bit_to_bool(masterBoardData->DigitaOutputBits, i));
                                ROS_INFO("Output State %d: %d", 18 + i, robotState.get_IO(18+i));
                            }

                            // TODO: Read Tool inputs and outputs??? The following worked in the past but not with CB3.2

						    // IOS
						    // 0-7 digital input, 8-15 configurable input, 16-17 tool input, 18-25 digital output, 26-33 configurable output, 34-35 tool output
						    /* for (int i= 0; i< 8; i++) robotState.set_IO(i, packet->bit_to_bool(dataPackageContent, 452, i%8));
						    for (int i= 8; i<16; i++) robotState.set_IO(i, packet->bit_to_bool(dataPackageContent, 451, i%8));
						    for (int i=16; i<18; i++) robotState.set_IO(i, packet->bit_to_bool(dataPackageContent, 450, i%2));
						    for (int i=18; i<26; i++) robotState.set_IO(i, packet->bit_to_bool(dataPackageContent, 456, i%8));
						    for (int i=26; i<34; i++) robotState.set_IO(i, packet->bit_to_bool(dataPackageContent, 455, i%8));
						    for (int i=34; i<36; i++) robotState.set_IO(i, packet->bit_to_bool(dataPackageContent, 454, i%2)); */


                        break;
                        }
                        case 4:
                        {
                            Packet_port30002::CartesianInfo *cartesianInfo = (Packet_port30002::CartesianInfo*)&dataPackageContent[bytepointer+sizeof(Packet_port30002::PacketHeader)];
                            ROS_DEBUG("CartesianInfo Package received");
                            cartesianInfo->fixByteOrder();
                            cartesianPosition.setValues(
                                        cartesianInfo->X_Tool,
                                        cartesianInfo->Y_Tool,
                                        cartesianInfo->Z_Tool,
                                        cartesianInfo->Rx,
                                        cartesianInfo->Ry,
                                        cartesianInfo->Rz);

                        break;
                        }
                    }

                    bytepointer+=packageHeader->packageLength;
                }

                robotState.setJointPosition(jointPosition);
                robotState.setJointVelocity(jointVelocity);
                robotState.setCartesianPosition(cartesianPosition);

                notifyListeners(robotState);
            }
            else if (port == REALTIME)
            {
                //TODO support port
                ROS_WARN("port %i not supported", port);
            }

            if (readFrequency > 0)
            {
                rate.sleep();
            }
        }
        catch (std::exception& e)
        {
            ROS_WARN_NAMED("connector", "error in read socket thread: %s", e.what());
        }
    }

    ROS_DEBUG_NAMED("connector", "exit readSocketWorker thread");
}

void Connector::writeSocketWorker()
{
    ros::Rate rate(writeFrequency);

    while(runWriteSocketThread && socket.is_open())
    {
        try
        {
            mutexCommandQueue.lock();

            if (!commandQueue.empty())
            {
                Command* command = commandQueue.front();
                commandQueue.pop();

                std::string commandStr = command->getCommandString();

                delete command;

                ROS_DEBUG_NAMED("connector", "socket write: send command to robot controller: %s", commandStr.c_str());

                boost::system::error_code error;
                size_t length = socket.write_some(boost::asio::buffer(commandStr), error);
            }

            mutexCommandQueue.unlock();

            if (writeFrequency > 0)
            {
                rate.sleep();
            }
        }
        catch (std::exception& e)
        {
            ROS_WARN_NAMED("connector", "error in write socket thread: %s", e.what());
        }
    }

    ROS_DEBUG_NAMED("connector", "exit writeSocketWorker thread");
}

inline std::string Connector::hexString(char data[], int length)
{
    std::stringstream hex;
    for (int i = 0; i < length; i++)
    {
        hex <<std::hex <<std::setfill('0') <<std::setw(2) <<(unsigned short)(unsigned char)data[i] <<" ";
    }

    return hex.str();
}
