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
// This file contains the driver logic
// ----------------------------------------------------------------------------

#ifndef DRIVER_H_
#define DRIVER_H_

#include <string.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <robot_movement_interface/EulerFrame.h>
#include <robot_movement_interface/Command.h>
#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/Result.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <ur_driver/DigIOAction.h>
#include <ur_driver/DigIOArrayAction.h>

#include <std_srvs/Empty.h>

#include <connector.h>

#include <boost/thread.hpp>
#include <math.h>
#include <assert.h>

namespace ur_driver
{

    /**
     * Configuration parameters which can be defined in a yaml file.
     */
    class Configuration
    {
        public:
            int loggerLevel;
            std::string host;
            int port;
            bool isDummy;
            std::vector<std::string> jointNames;
            std::string robotBaseFrameName;
            std::string robotFlangeFrameName;
            std::string robotTcpFrameName;
            double robotReadFrequency;
            double robotWriteFrequency;
            double publishStateFrequency;
            double velocity;
            double acceleration;
            double angleTolerance;
            double maxLinearVelocity;
            double maxAngularVelocity;

            /**
             * Constructor.
             * @param nodeHandle
             */
            Configuration(ros::NodeHandle& nodeHandle);

            /**
             * Descructor.
             */
            ~Configuration();
    };

    /**
     * ROS driver and interface for the universal robot.
     */
    class Driver
    {
        public:
            /**
             * Constructor.
             */
            Driver();

            /**
             * Destructor.
             */
            ~Driver();

            /**
             * Starts a ROS spinner and blocks until the shutdown signal was received.
             */
            void spin();

        private:
       
            ros::NodeHandle nodeHandle;

            bool runRobotStatePublishThread;
            boost::thread robotStatePublishThread;
            boost::thread signalThread;

            static bool shutdownSignal;

            boost::mutex mutexRobotMovement;

            bool stopCommandReceived;

            /*
             * configuration
             */
            Configuration configuration;

            RobotState lastRobotState;
            tf::TransformListener tfListener;
            tf::TransformBroadcaster tfBroadcaster;

            /*
             * Connector
             */
            Connector connector;

            /*
             * Interface Input
             */
            /*ros::ServiceServer commandHomeSubscriber;
            ros::ServiceServer commandJointStopSubscriber;
            ros::ServiceServer commandCartesianStopSubscriber;

            ros::Subscriber jointVelocitySubscriber;
            ros::Subscriber cartesianVelocitySubscriber;

            actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> jointPositionServer;*/
            actionlib::SimpleActionServer<ur_driver::DigIOAction> digitalIOServer;
            actionlib::SimpleActionServer<ur_driver::DigIOArrayAction> digitalIOArrayServer;

            /*
             * Robot Movement Action v2 (2 topics) -> paq@ipa.fhg.de
             */
            boost::thread commandThread;
            boost::mutex commandMutex;
            std::vector<robot_movement_interface::Command> commandList;
            robot_movement_interface::Command commandActive;
			bool isLastCommand;
			robot_movement_interface::Command lastCommand;
            ros::Time lastCommandExecutionTime;

            bool isCommandActive;

            ros::Subscriber commandListSubscriber;
            ros::Publisher commandResultPublisher;

            /*
             * Interface Output
             */
            ros::Publisher jointStatePublisher;
            ros::Publisher poseStatePublisher;
            ros::Publisher toolFrameStatePublisher;

            /**
             * Callback for receiving a home command request from a client. (service server)
             * The home command will move the robot into the home position.
             * @param req
             * @param res
             * @return
             */
            //bool homeCommandCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            /**
             * Callback for receiving a joint stop command request from a client. (service server)
             * The joint stop command will stop any robot movement linear in joint space.
             * @param req
             * @param res
             * @return
             */
            //bool stopJointCommandCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            /**
             * Callback for receiving a cartesian stop command request from a client. (service server)
             * The cartesian stop command will stop any robot movement linear in tool space.
             * @param req
             * @param res
             * @return
             */
            //bool stopCartesianCommandCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            /**
             * Callback for receiving a joint velocity goal from a client. (message subscriber)
             * Apply a joint velocity to the joints over a specific time. The time is depended of the the frequency defined
             * in Configuration::robotWriteFrequency.
             * @param msg
             */
            //void jointVelocityCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

            /**
             * Callback for receiving a cartesian velocity goal from a client. (message subscriber)
             * Apply a joint velocity to the tool in a specific frame over a specific time. If no frame is specified, the default frame
             * (base frame) is used. The time is depended of the the frequency defined in Configuration::robotWriteFrequency.
             * @param msg
             */
            //void cartesianVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

            /**
             * Callback for receiving a joint position goal from a client. (action server)
             * Set the pose of the robot using the joint position.
             * @param goal
             */
            //void jointPositionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

            /**
             * Callback for receiving a cartesian position goal from a client. (action server)
             * Set the pose of the robot using the tool position in a specific frame. If no frame is specified, the default frame (base
             * frame) is used.
             * @param goal
             */
            // void cartesianPositionCallback(const ipa325_msgs::RobotMovementGoalConstPtr &goal);

            /**
             * Callback for receiving a robot movement action v2 command list
             * @param commandList
             */
            void commandListCallback(const robot_movement_interface::CommandListConstPtr &msg);

            /**
             * Callback for receiving a digital IO goal from a client. (action server)
             * Set digital IO of the robot.
             * @param goal
             */
            void executeDigIo(const ur_driver::DigIOGoalConstPtr &goal);

            /**
             * Worker thread for publishing the robot state.
             */
            void robotStatePublishWorker();

            /**
             * Callback for receiving continuously robot state updates from the connector.
             * @param robotState
             */
            void robotStateListener(const RobotState& robotState);

            /**
             * Callback for receiving signal. When SIGINT was received shutdown everything.
             * @param signal
             */
            static void signalHandler(int signal);

            /**
             * Worker thread for robot movement action v2
             */
            void commandThreadWorker();
            bool isCommandFinished(robot_movement_interface::Command command, int *result);
			int processCommand(robot_movement_interface::Command command, ur_driver::Command * result);  
			void replaceQuaternions(std::vector<robot_movement_interface::Command> & list);
			void transformQuaternionToEulerIntrinsicZYX(float qx, float qy, float qz, float qw, float * z, float * y, float * x );
            std::vector<float> transformQuaternionToEulerIntrinsicZYX(std::vector<float> quaternion);

            void executeDigIoArray(const ur_driver::DigIOArrayGoalConstPtr &goal);
    };
}

#endif
