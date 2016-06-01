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

#ifndef CONNECTOR_H_
#define CONNECTOR_H_

#include <ros/ros.h>

#include <stdio.h>
#include <stdint.h>
#include <endian.h>
#include <semaphore.h>

#include <string>
#include <queue>
#include <stdexcept>
#include <cstdarg>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include <utils.h>
#include <command.h>
#include <dummy.h>

namespace ur_driver
{
    //=================================================================
    // Packages
    //=================================================================
    inline double bedtoh(const double &x)
    {
        double temp;
        *((uint64_t*)(&temp)) = be64toh(*((uint64_t*)&x));
        return temp;
    }

    inline float beftoh(const float &x)
    {
        float temp;
        *((uint32_t*)(&temp)) = be32toh(*((uint32_t*)&x));
        return temp;
    }

    /**
     * Packet structure on port 30002
     */
    class Packet_port30002
    {
        public:
            class PacketHeader
            {
                public:
                    int packageLength;
                    unsigned char packageType;

                public:
                    void fixByteOrder()
                    {
                        packageLength = be32toh(packageLength);
                    }
            }__attribute__((packed));

            class ToolData
            {
                public:
                    unsigned char stuff[37-5];

                public:
                    void fixByteOrder()
                    {
                        // TODO
                    }
            }__attribute__((packed));

            class MasterboardData
            {
                public:
                    int DigitalnputBits;
                    int DigitaOutputBits;

                public:
                    void fixByteOrder()
                    {
                        DigitalnputBits = be32toh(DigitalnputBits);
                        DigitaOutputBits = be32toh(DigitaOutputBits);
                    }

                    // returns true if bit number pos of a given byte is true or false if not
                    bool bit_to_bool(int byte, int pos){
                        return (byte >> pos) & 1;
                    }
            }__attribute__((packed));

            class CartesianInfo
            {
                public:
                    double X_Tool;// vector, X-value
                    double Y_Tool;// vector, Y-value
                    double Z_Tool;// vector, Z-value
                    double Rx; //Rx: Rotation vector representation of the tool orientation
                    double Ry; //Ry: Rotation vector representation of the tool orientation
                    double Rz; //Rz: Rotation vector representation of the tool orientation

                    double TCPOffsetX; //TCP offset, X-value
                    double TCPOffsetY; //TCP offset, Y-value
                    double TCPOffsetZ; //TCP offset, Z-value
                    double TCPOffsetRX; //TCP offset, Rx-value (Rotation vector representation of TCP orientation)
                    double TCPOffsetRY; //TCP offset, Ry-value (Rotation vector representation of TCP orientation)
                    double TCPOffsetRZ;	//TCP offset, Rz-value (Rotation vector representation of TCP orientation)

                public:
                    void fixByteOrder()
                    {
                        X_Tool = bedtoh(X_Tool);
                        Y_Tool = bedtoh(Y_Tool);
                        Z_Tool = bedtoh(Z_Tool);
                        Rx     = bedtoh(Rx);
                        Ry     = bedtoh(Ry);
                        Rz     = bedtoh(Rz);

                        TCPOffsetX     = bedtoh(TCPOffsetX);
                        TCPOffsetY     = bedtoh(TCPOffsetY);
                        TCPOffsetZ     = bedtoh(TCPOffsetZ);
                        TCPOffsetRX     = bedtoh(TCPOffsetRX);
                        TCPOffsetRY     = bedtoh(TCPOffsetRY);
                        TCPOffsetRZ     = bedtoh(TCPOffsetRZ);
                    }
            }__attribute__((packed));

            class RobotMode
            {
                public:
                    unsigned long long timeStamp;
                    bool isPhysicalRobotConnected;
                    bool isRealRobotEnabled;
                    bool isRobotPowerOn;
                    bool isEmergencyStopped;
                    bool isSecurityStopped;
                    bool isProgramRunning;
                    bool isProgramPaused;
                    unsigned char Robot_Mode;//   See table Robot Modes
                    unsigned char ControlMode; //new in CB3
                    double Speed_Fraction;
                    double SpeedScaling; //new in CB3

                public:
                    void fixByteOrder()
                    {
                        timeStamp = be64toh(timeStamp);
                        Speed_Fraction = bedtoh(Speed_Fraction);
                        SpeedScaling = bedtoh(SpeedScaling);
                    }
            } __attribute__((packed));

            class Joint
            {
                public:
                    double q_act;
                    double q_tar;
                    double qd_act;
                    float current;
                    float voltage;
                    float temperature;
                    float unknown; //obsolete
                    unsigned char JointMode; //new in CB3

                public:
                    void fixByteOrder()
                    {
                        q_act       = bedtoh(q_act);
                        q_tar       = bedtoh(q_tar);
                        qd_act      = bedtoh(qd_act);
                        current     = beftoh(current);
                        voltage     = beftoh(voltage);
                        temperature = beftoh(temperature);
                        unknown     = beftoh(unknown);
                    }

            } __attribute__((packed));

            //double time;
            unsigned char robotMessageType;

            // Get info out here!!!
            PacketHeader robotModeHeader;
            RobotMode robotMode;
            PacketHeader jointsHeader;
            Joint joint[6];

            PacketHeader cartesianInfoHeader;
            CartesianInfo cartesianInfo;

            PacketHeader masterBoardHeader;
            MasterboardData masterboardData;

            PacketHeader toolDataHeader;
            ToolData toolData;

        public:
            void fixByteOrder()
            {
                robotModeHeader.fixByteOrder();
                robotMode.fixByteOrder();
                jointsHeader.fixByteOrder();
                for (int ii=0; ii < 6; ++ii)
                {
                    joint[ii].fixByteOrder();
                }
                toolDataHeader.fixByteOrder();
                toolData.fixByteOrder();
                masterBoardHeader.fixByteOrder();
                masterboardData.fixByteOrder();
                cartesianInfoHeader.fixByteOrder();
                cartesianInfo.fixByteOrder();
            }

            // returns true if bit number pos of a given byte is true or false if not
            bool bit_to_bool(char * byteArray, int byteNum, int pos){
                return (byteArray[byteNum] >> pos) & 1;
            }
    } __attribute__((packed));

    /**
     * Packet structure on port 30003
     */
    class Packet_port30003
    {
        public:
            double time;
            double q_target[6];
            double qd_target[6];
            double qdd_target[6];
            double I_target[6];
            double M_target[6];

            double q_act[6];
            double qd_act[6];
            double I_act[6];

            double tool_acc[3];
            double unused[15];

            double tcp_force[6];

            double tool_pose[6];
            double tool_vel[6];

            int64_t dig_in_bits; // int64_t bitwise encoded

            double motor_temp[6];
            double controller_timer;
            double testValue;
            double robot_mode;
            double joint_modes[6];



            void fixByteOrder()
            {
                time = bedtoh(time);
                controller_timer = bedtoh(controller_timer);
                testValue = bedtoh(testValue);
                robot_mode = bedtoh(robot_mode);

                for (int i=0; i<3; i++)
                {
                    tool_acc[i] = bedtoh(tool_acc[i]);
                }

                for (int i=0; i < 6; i++)
                {
                    q_target[i] = bedtoh(q_target[i]);
                    qd_target[i] = bedtoh(qd_target[i]);
                    qdd_target[i] = bedtoh(qdd_target[i]);
                    I_target[i] = bedtoh(I_target[i]);
                    M_target[i] = bedtoh(M_target[i]);
                    q_act[i] = bedtoh(q_act[i]);
                    qd_act[i] = bedtoh(qd_act[i]);
                    I_act[i] = bedtoh(I_act[i]);
                    tcp_force[i] = bedtoh(tcp_force[i]);
                    tool_pose[i] = bedtoh(tool_pose[i]);
                    tool_vel[i] = bedtoh(tool_vel[i]);

                    motor_temp[i] = bedtoh(motor_temp[i]);
                    joint_modes[i] = bedtoh(joint_modes[i]);
                }
            }

    }__attribute__((packed));

    //=================================================================
    // RobotState
    //=================================================================
    class RobotState
    {
        public:
            /**
             * Constructor.
             */
            RobotState();

            /**
             * Constructor.
             * @param jointPosition
             * @param jointVelocity
             * @param cartesianPosition
             */
            RobotState(const JointPosition& jointPosition, const JointVelocity& jointVelocity, const CartesianPosition& cartesianPosition);

            /**
             * Get joint position.
             * @return
             */
            JointPosition& getJointPosition();

            /**
             * Set joint position.
             * @param jointPosition
             */
            void setJointPosition(const JointPosition& jointPosition);

            /**
             * Get joint velocity.
             * @return
             */
            JointVelocity& getJointVelocity();

            /**
             * Set joint velocity.
             * @param jointVelocity
             */
            void setJointVelocity(const JointVelocity& jointVelocity);

            /**
             * Get cartesian position.
             * @return
             */
            CartesianPosition& getCartesianPosition();

            /**
             * Set cartesian position.
             * @param cartesianPosition
             */
            void setCartesianPosition(const CartesianPosition& cartesianPosition);

            bool get_IO(int i){
                return IOS[i];
            }
            void set_IO(int i, bool v){
                IOS[i] = v;
            }

            bool isUrProgramRunning;
            bool isUrProgramPaused;

    private:
            JointPosition jointPosition;
            JointVelocity jointVelocity;
            CartesianPosition cartesianPosition;

            bool IOS[36]; // 0-7 digital input, 8-15 configurable input, 16-17 tool input, 18-25 digital output, 26-33 configurable output, 34-35 tool output
    };

    //=================================================================
    // Connector
    //=================================================================
    /**
     * The Connector class handles the TCP connection to the robot controller.
     * In the reading direction the robot status will be read continuously.
     * In the writing direction the commands will be send.
     */
    class Connector
    {
        public:
            typedef enum InterfacePort
            {
                PRIMARY = 30001,
                SECONDARY = 30002,
                REALTIME = 30003
            } InterfacePort;

            /**
             * Constructor
             */
            Connector();

            /**
             * Destructor
             */
            ~Connector();

            /**
             * Add a command to the command queue.
             * @param command
             */
            void addCommand(Command* command);

            /**
             * Connect to the robot controller on the given host address and port.
             * Note: Currently only the connection to port 30002 is supported. (TODO)
             * @param host IP or DNS name of the robot controller.
             * @param port Port number for the connection. Usually 30001, 30002, 30003.
             * @param isDummy
             * @param readFrequency
             * @param writeFrequency
             */
            void connect(std::string host, int port, bool isDummy, double readFrequency, double writeFrequency);

            /**
             * Disconnect from the robot controller.
             */
            void disconnect();

            /**
             * Add a listener to get notified when a new robot status was received from the robot controller via Ethernet.
             * @param member
             * @param object
             */
            template <typename T>
            void addRobotStateListener(void (T::*member)(const RobotState&), T* object)
            {
                signalRobotState.connect(boost::bind(member, object, _1));
            }

            /**
             * Remove a listener to get notified when a new robot status was received from the robot controller via Ethernet.
             * @param member
             * @param object
             */
            template <typename T>
            void removeRobotStateListener(void (T::*member)(const RobotState&), T* object)
            {
                signalRobotState.disconnect(boost::bind(member, object, _1));
            }

            /**
             * Notify all listeners with a robot state.
             * @param robotState The robot state to send to all listeners.
             */
            void notifyListeners(RobotState& robotState);

        private:
            Dummy dummy;

            /*
             * socket stuff
             */
            bool runConnectSocketThread;
            bool runReadSocketThread;
            bool runWriteSocketThread;
            boost::thread connectSocketThread;
            boost::thread readSocketThread;
            boost::thread writeSocketThread;

            boost::asio::io_service io;
            boost::asio::ip::tcp::socket socket;

            bool isRunning;

            /*
             * configuration
             */
            std::string host;
            int port;
            bool isDummy;
            double readFrequency;
            double writeFrequency;

            /*
             * signals
             */
            boost::signals2::signal<void (const RobotState&)> signalRobotState;

            /*
             * command queue
             */
            std::queue<Command*> commandQueue;

            /*
             * synchronization
             */
            boost::mutex mutexCommandQueue;
            boost::mutex mutexStartStop;

            /**
             * Worker thread for establishing a connection to the robot controller.
             * Automatically reconnect if a connection failed or was lost.
             */
            void connectSocketWorker();

            /**
             * Worker thread for reading from the socket.
             */
            void readSocketWorker();

            /**
             * Worker thread for writing to the socket.
             */
            void writeSocketWorker();

            /**
             * Helper function to create a hex string from a char array.
             * @param data
             * @param length
             * @return
             */
            inline std::string hexString(char data[], int length);
    };
}

#endif
