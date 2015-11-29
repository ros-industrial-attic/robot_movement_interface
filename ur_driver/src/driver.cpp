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

#include <driver.h>
#include <utils.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <ros/console.h>
#include <tf/tf.h>
#include <signal.h>
#include <errno.h>

using namespace std;
using namespace ur_driver;

//=================================================================
// Configuration
//=================================================================
Configuration::Configuration(ros::NodeHandle& nodeHandle)
{
    ROS_INFO_NAMED("driver", "load configuration");

    //logger level [DEBUG=0, INFO=1, WARN=2, ERROR=3, FATAL=4]
    nodeHandle.param<int>("loggerLevel", loggerLevel, 1);
    ROS_DEBUG_NAMED("driver", "loggerLevel=%i", loggerLevel);

    //IP or DNS name of the robot controller
    nodeHandle.param<string>("host", host, "localhost");
    ROS_DEBUG_NAMED("driver", "host=%s", host.c_str());

    //port of the robot controller
    nodeHandle.param<int>("port", port, 30002);
    ROS_DEBUG_NAMED("driver", "port=%i", port);

    //use a dummy server for connection. Note: you should use the UR simulation instead (http://support.universal-robots.com/Downloads/PolyScopeURSim)
    nodeHandle.param<bool>("isDummy", isDummy, true);
    ROS_DEBUG_NAMED("driver", "isDummy=%s", (isDummy) ? "true" : "false");

    //joint names of the (non fixed) joints
    XmlRpc::XmlRpcValue value;
    nodeHandle.getParam("jointNames", value);
    for(int i = 0; i < value.size(); i++)
    {
        jointNames.push_back(value[i]);
    }

    if (jointNames.size() == 0)
    {
        jointNames.push_back("shoulder_pan_joint");
        jointNames.push_back("shoulder_lift_joint");
        jointNames.push_back("elbow_joint");
        jointNames.push_back("wrist_1_joint");
        jointNames.push_back("wrist_2_joint");
        jointNames.push_back("wrist_3_joint");
    }

    //the name of the base frame
    nodeHandle.param<string>("robotBaseFrameName", robotBaseFrameName, "ur_base");
    ROS_DEBUG_NAMED("driver", "robotBaseFrameName=%s", robotBaseFrameName.c_str());

    //the name of the flange frame
    nodeHandle.param<string>("robotFlangeFrameName", robotFlangeFrameName, "ur_flange");
    ROS_DEBUG_NAMED("driver", "robotFlangeFrameName=%s", robotFlangeFrameName.c_str());

    //the name of the TCP frame
    nodeHandle.param<string>("robotTcpFrameName", robotTcpFrameName, "ur_flange");
    ROS_DEBUG_NAMED("driver", "robotTcpFrameName=%s", robotTcpFrameName.c_str());

    //the frequency with which the driver will read the robot state from the robot controller
    nodeHandle.param<double>("robotReadFrequency", robotReadFrequency, 20);
    ROS_DEBUG_NAMED("driver", "robotReadFrequency=%f", robotReadFrequency);

    //the frequency with which the driver will write commands to the robot controller
    nodeHandle.param<double>("robotWriteFrequency", robotWriteFrequency, 20);
    ROS_DEBUG_NAMED("driver", "robotWriteFrequency=%f", robotWriteFrequency);

    //publish frequency of the robot state
    nodeHandle.param<double>("publishStateFrequency", publishStateFrequency, 20);
    ROS_DEBUG_NAMED("driver", "publishStateFrequency=%f", publishStateFrequency);

    //velocity
    //TODO really needed? -> use parameter in message/action
    nodeHandle.param<double>("velocity", velocity, 0.1);
    ROS_DEBUG_NAMED("driver", "velocity=%f", velocity);

    //acceleration
    //TODO really needed? -> use parameter in message/action
    nodeHandle.param<double>("acceleration", acceleration, 3.0);
    ROS_DEBUG_NAMED("driver", "acceleration=%f", acceleration);

    //angle tolerance when target position is reached
    //TODO really needed? -> use parameter in message/action
    nodeHandle.param<double>("angleTolerance", angleTolerance, 0.02);
    ROS_DEBUG_NAMED("driver", "angleTolerance=%f", angleTolerance);

    //limit for linear velocity
    nodeHandle.param<double>("maxLinearVelocity", maxLinearVelocity, 1.0);
    ROS_DEBUG_NAMED("driver", "maxLinearVelocity=%f", maxLinearVelocity);

    //limit for angular velocity
    nodeHandle.param<double>("maxAngularVelocity", maxAngularVelocity, 0.5);
    ROS_DEBUG_NAMED("driver", "maxAngularVelocity=%f", maxAngularVelocity);
}

Configuration::~Configuration()
{

}

//=================================================================
// Driver
//=================================================================
bool Driver::shutdownSignal = false;

Driver::Driver() :
    configuration(nodeHandle),
    /*jointPositionServer(nodeHandle, "joint_pos", boost::bind(&Driver::jointPositionCallback, this, _1), false),
    /*cartesianPositionServer(nodeHandle, "cartesian_pos", boost::bind(&Driver::cartesianPositionCallback, this, _1), false),*/
    digitalIOServer(nodeHandle, "digital_io", boost::bind(&Driver::executeDigIo, this, _1), false),
    digitalIOArrayServer(nodeHandle, "digital_io_array", boost::bind(&Driver::executeDigIoArray, this, _1), false)
    /*,stopCommandReceived(false)*/
{
    //logger level
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, (ros::console::Level)configuration.loggerLevel);

    //signal handler
    signal((int) SIGINT, Driver::signalHandler);

    //setup input interface
    //jointVelocitySubscriber = nodeHandle.subscribe("joint_vel", 1, &Driver::jointVelocityCallback, this);
    //cartesianVelocitySubscriber = nodeHandle.subscribe("cartesian_vel", 1, &Driver::cartesianVelocityCallback, this);

    //commandHomeSubscriber = nodeHandle.advertiseService("cmd_home", &Driver::homeCommandCallback, this);
    //commandJointStopSubscriber = nodeHandle.advertiseService("cmd_stop_joint", &Driver::stopJointCommandCallback, this);
    //commandCartesianStopSubscriber = nodeHandle.advertiseService("cmd_stop_cartesian", &Driver::stopCartesianCommandCallback, this);

    //jointPositionServer.start();
    //cartesianPositionServer.start();
    digitalIOServer.start();
    digitalIOArrayServer.start();

    // Robot Movement Interface
    isCommandActive = false;
	isLastCommand = false;
    commandResultPublisher = nodeHandle.advertise<robot_movement_interface::Result>("command_result", 1);
    commandListSubscriber = nodeHandle.subscribe("command_list", 1, &Driver::commandListCallback, this);
    commandThread = boost::thread(&Driver::commandThreadWorker, this); // start commander

    //setup output interface
    jointStatePublisher = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);
    poseStatePublisher = nodeHandle.advertise<geometry_msgs::Pose>("pose_state", 1);
    toolFrameStatePublisher = nodeHandle.advertise<robot_movement_interface::EulerFrame>("tool_frame", 1);

    //start publisher
    runRobotStatePublishThread = true;
    robotStatePublishThread = boost::thread(&Driver::robotStatePublishWorker, this);

    //connect to robot controller
    connector.connect(configuration.host, configuration.port, configuration.isDummy, configuration.robotReadFrequency, configuration.robotWriteFrequency);
    connector.addRobotStateListener(&Driver::robotStateListener, this);

    ROS_INFO_NAMED("driver", "driver initialized");
}

Driver::~Driver()
{
    //stop publisher
    runRobotStatePublishThread = false;
    robotStatePublishThread.join();
}

void Driver::spin()
{
    //spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();

    //wait for shutdown
    ros::Rate rate(10);
    while(ros::ok() && !shutdownSignal)
    {
        rate.sleep();
    }

    delete this;
    ros::spinOnce();
}

/*
bool Driver::homeCommandCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO_NAMED("driver", "home command received");

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("joint_pos", false);

    bool success = client.waitForServer(ros::Duration(5.0));

    if (success)
    {
        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.joint_names.push_back("shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("elbow_joint");
        goal.trajectory.joint_names.push_back("wrist_1_joint");
        goal.trajectory.joint_names.push_back("wrist_2_joint");
        goal.trajectory.joint_names.push_back("wrist_3_joint");

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0);
        point.positions.push_back(-M_PI_2);
        point.positions.push_back(0);
        point.positions.push_back(-M_PI_2);
        point.positions.push_back(0);
        point.positions.push_back(0);

        goal.trajectory.points.push_back(point);

        client.sendGoal(goal);

        success = client.waitForResult(ros::Duration(60.0));
        success = success && client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
    else
    {
        ROS_ERROR("timeout waitForServer");
    }

    return success;
}

bool Driver::stopJointCommandCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO_NAMED("driver", "stop joint command received");

    connector.addCommand(new CommandJointStop(configuration.acceleration));
    stopCommandReceived = true;

    return true;
}

bool Driver::stopCartesianCommandCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO_NAMED("driver", "stop cartesian command received");

    connector.addCommand(new CommandCartesianStop(configuration.acceleration));
    stopCommandReceived = true;

    return true;
}

void Driver::jointVelocityCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    ROS_INFO_NAMED("driver", "received joint velocity");

    //get indices of the joint names
    std::vector<int> jointIndex(6, -1);
    for (unsigned int i = 0; i < msg->joint_names.size(); i++)
    {
        for (unsigned int j = 0; j < configuration.jointNames.size(); j++)
        {
            if (msg->joint_names[i] == configuration.jointNames[j])
            {
                jointIndex[i] = j;
                break;
            }
        }
    }

    //check if all joint names were defined
    for (int i = 0; i < 6; i++)
    {
        if (jointIndex[i] == -1)
        {
            ROS_WARN_NAMED("driver", "missing joint name of at least one joint. All points will be ignored");

            return;
        }
    }

    //create a command for all points
    for (int i = 0; i < msg->points.size(); i++)
    {
        if (msg->points[i].velocities.size() != msg->joint_names.size())
        {
            ROS_WARN_NAMED("driver", "velocities count doesn't match joint names count. This point will be ignored");

            continue;
        }

        double acceleration = configuration.acceleration;

        //copy velocity
        JointVelocity jointVelocity(configuration.jointNames.size());
        bool stop = true;
        for (int j = 0; j < msg->points[i].velocities.size(); j++)
        {
            jointVelocity[jointIndex[j]] = msg->points[0].velocities[j];

            //limit velocity
            if (jointVelocity[jointIndex[j]] > configuration.maxAngularVelocity)
            {
                ROS_WARN_NAMED("driver", "angular velocity limit on joint \"%s\" reached. Limit joint velocity to %f", msg->joint_names[j].c_str(), configuration.maxAngularVelocity);
                jointVelocity[jointIndex[j]] = configuration.maxAngularVelocity;
            }

            //zero velocity
            if (jointVelocity[jointIndex[j]] != 0.0)
            {
                stop = false;
            }
        }

        ROS_INFO_NAMED("driver", "acceleration: %f, velocity: %s", acceleration, jointVelocity.toString().c_str());

        //add command to the command queue
        Command* command;
        if (stop)
        {
            command = new CommandJointStop(acceleration);
        }
        else
        {
            //TODO wieso "mal 2"?
            command = new CommandJointVelocity(jointVelocity, configuration.acceleration, (1.0 / configuration.robotWriteFrequency) * 2);
        }
        connector.addCommand(command);
    }
}

void Driver::cartesianVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    ROS_INFO_NAMED("driver", "received cartesian velocity");

    std::string targetFrameName = msg->header.frame_id;

    //copy velocity
    CartesianVelocity cartesianVelocity;
    cartesianVelocity.x() = msg->twist.linear.x;
    cartesianVelocity.y() = msg->twist.linear.y;
    cartesianVelocity.z() = msg->twist.linear.z;
    cartesianVelocity.rx() = msg->twist.angular.x;
    cartesianVelocity.ry() = msg->twist.angular.y;
    cartesianVelocity.rz() = msg->twist.angular.z;

    //transform velocity from target frame into base frame
    if (targetFrameName != "")
    {
        try
        {
            ROS_INFO_NAMED("driver", "transform cartesian velocity into frame \"%s\"", targetFrameName.c_str());

            //TODO transformation from flange frame into tool frame needed?
            //TODO test behavior if target frame is rotate relative to the flange frame
            tf::StampedTransform transformTarget2Flange;
            tf::StampedTransform transformFlange2Base;
            tfListener.lookupTransform(configuration.robotFlangeFrameName, targetFrameName, ros::Time(0), transformTarget2Flange);
            tfListener.lookupTransform(configuration.robotBaseFrameName, configuration.robotFlangeFrameName, ros::Time(0), transformFlange2Base);

            //target frame
            tf::Vector3 linearVelocityTarget(cartesianVelocity.x(), cartesianVelocity.y(),cartesianVelocity.z());
            tf::Vector3 angularVelocityTarget(cartesianVelocity.rx(), cartesianVelocity.ry(),cartesianVelocity.rz());

            //transform velocities into flange frame
            tf::Vector3 linearVelocityFlange = transformTarget2Flange.getBasis() * linearVelocityTarget + angularVelocityTarget.cross(transformTarget2Flange.inverse().getOrigin());
            tf::Vector3 angularVelocityFlange = transformTarget2Flange.getBasis() * angularVelocityTarget;

            //transform velocities into base frame
            tf::Vector3 linearVelocityBase = transformFlange2Base.getBasis() * linearVelocityFlange;
            tf::Vector3 angularVelocityBase = transformFlange2Base.getBasis() * angularVelocityFlange;

            cartesianVelocity.x() = linearVelocityBase.x();
            cartesianVelocity.y() = linearVelocityBase.y();
            cartesianVelocity.z() = linearVelocityBase.z();
            cartesianVelocity.rx() = angularVelocityBase.x();
            cartesianVelocity.ry() = angularVelocityBase.y();
            cartesianVelocity.rz() = angularVelocityBase.z();
        }
        catch (tf::TransformException& e)
        {
            ROS_ERROR_NAMED("driver", "transformation of velocity into base frame failed: %s", e.what());

            return;
        }
    }

    double acceleration = configuration.acceleration;

    //limit velocity
    for (int j = 0; j < 3; j++)
    {
        if (cartesianVelocity[j] > configuration.maxLinearVelocity)
        {
            ROS_WARN_NAMED("driver", "linear velocity limit reached. Limit linear velocity to %f", configuration.maxLinearVelocity);
            cartesianVelocity[j] = configuration.maxLinearVelocity;
        }
    }

    for (int j = 3; j < 6; j++)
    {
        if (cartesianVelocity[j] > configuration.maxAngularVelocity)
        {
            ROS_WARN_NAMED("driver", "angular velocity limit reached. Limit joint velocity to %f", configuration.maxAngularVelocity);
            cartesianVelocity[j] = configuration.maxAngularVelocity;
        }
    }

    //check if velocities all zero
    bool stop = cartesianVelocity.x() == 0.0 && cartesianVelocity.y() == 0.0 && cartesianVelocity.z() == 0.0 && cartesianVelocity.rx() == 0 && cartesianVelocity.ry() == 0 && cartesianVelocity.rz() == 0;

    ROS_INFO_NAMED("driver", "acceleration: %f, velocity: %s", acceleration, cartesianVelocity.toString().c_str());

    //add command to the command queue
    Command* command;
    if (stop)
    {
        command = new CommandCartesianStop(acceleration);
    }
    else
    {
        //TODO wieso "mal 2"?
        command = new CommandCartesianVelocity(cartesianVelocity, acceleration, (1.0 / configuration.robotWriteFrequency) * 2);
    }
    connector.addCommand(command);
}

void Driver::jointPositionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    ROS_INFO_NAMED("driver", "received joint position");

    mutexRobotMovement.lock();

    control_msgs::FollowJointTrajectoryResult result;
    control_msgs::FollowJointTrajectoryFeedback feedback;

    trajectory_msgs::JointTrajectory trajectory = goal->trajectory;

    //get indices of the joint names
    std::vector<int> jointIndices(configuration.jointNames.size(), -1);
    for (unsigned int i = 0; i < trajectory.joint_names.size(); i++)
    {
        for (unsigned int j = 0; j < configuration.jointNames.size(); j++)
        {
            if (trajectory.joint_names[i] == configuration.jointNames[j])
            {
                jointIndices[i] = j;
                break;
            }
        }
    }

    //check if all joint names were defined
    for (int i = 0; i < 6; i++)
    {
        if (jointIndices[i] == -1)
        {
            ROS_WARN_NAMED("driver", "missing joint name of at least one joint. All points will be ignored");
            ROS_WARN_NAMED("driver", "joint position result: Aborted");

            mutexRobotMovement.unlock();
            jointPositionServer.setAborted(result);

            return;
        }
    }

    //get goal tolerance
    std::vector<double> goalTolerance(goal->goal_tolerance.size());
    for (int i = 0; i < goal->goal_tolerance.size(); i++)
    {
        goalTolerance[jointIndices[i]] = goal->goal_tolerance[i].position;
    }

    if (goalTolerance.size() < trajectory.joint_names.size())
    {
        double defaultGoalTolerance = 0.01;
        ROS_WARN_NAMED("driver", "goal tolerance for all joints was not set. Set goal tolerance to default value: %f", defaultGoalTolerance);
        goalTolerance.resize(trajectory.joint_names.size(), defaultGoalTolerance);
    }

    //prepare feedback
    feedback.joint_names = trajectory.joint_names;
    feedback.actual.positions.resize(trajectory.joint_names.size());
    feedback.desired.positions.resize(trajectory.joint_names.size());
    feedback.error.positions.resize(trajectory.joint_names.size());

    //create a command for all points
    for (int i = 0; i < trajectory.points.size(); i++)
    {
        //verify point
        if (trajectory.points[i].positions.size() != trajectory.joint_names.size())
        {
            ROS_WARN_NAMED("driver", "positions count doesn't match joint names count. This point will be ignored");

            continue;
        }

        //copy position
        JointPosition jointPosition(configuration.jointNames.size());
        for (int j = 0; j < trajectory.points[i].positions.size(); j++)
        {
            jointPosition[jointIndices[j]] = trajectory.points[i].positions[j];
        }

        double velocity = configuration.velocity;
        double acceleration = configuration.acceleration;

        //limit velocity
        if (velocity > configuration.maxAngularVelocity)
        {
            ROS_WARN_NAMED("driver", "angular velocity limit reached. Limit joint velocity to %f", configuration.maxAngularVelocity);
            velocity = configuration.maxAngularVelocity;
        }

        ROS_INFO_NAMED("driver", "velocity: %f, acceleration: %f, position: %s", velocity, acceleration, jointPosition.toString().c_str());

        //add command to the command queue
        Command* command = new CommandJointPosition(jointPosition, velocity, acceleration);
        connector.addCommand(command);

        //send feedback while moving to target position
        ros::Rate rate(10);
        while(ros::ok() && !jointPositionServer.isPreemptRequested() && !shutdownSignal && !stopCommandReceived)
        {
            //set feedback
            for (int j = 0; j < trajectory.points[i].positions.size(); j++)
            {
                feedback.actual.positions[j] = lastRobotState.getJointPosition()[j];
                feedback.desired.positions[j] = jointPosition[j];
                feedback.error.positions[j] = feedback.desired.positions[j] - feedback.actual.positions[j];
                // ROS_INFO_NAMED("driver", "%s: %f", trajectory.joint_names[j].c_str(), feedback.error.positions[j]);
                // ROS_INFO_NAMED("driver", "actual: %f", feedback.actual.positions[j]);
                // ROS_INFO_NAMED("driver", "desired: %f", feedback.desired.positions[j]);
            }

            //check if target position is reached
            bool reached = true;
            
            for (int j = 0; j < feedback.error.positions.size(); j++)
            {
                if (fabs(feedback.error.positions[j]) > goalTolerance[j])
                {
                    reached = false;

                    break;
                }
            }

            //reached target position
            if (reached || configuration.isDummy)
            {
                ROS_INFO_NAMED("driver", "reached joint target position: %s", jointPosition.toString().c_str());

                break;
            }

            jointPositionServer.publishFeedback(feedback);

            rate.sleep();
        }
    }

    mutexRobotMovement.unlock();

    if (stopCommandReceived)
    {
        stopCommandReceived = false;

        ROS_INFO_NAMED("driver", "joint position result: Aborted");
        jointPositionServer.setAborted(result);
    }
    else if (jointPositionServer.isPreemptRequested())
    {
        ROS_INFO_NAMED("driver", "joint position result: Preempted");
        jointPositionServer.setPreempted(result);
    }
    else
    {
        ROS_INFO_NAMED("driver", "joint position result: Succeeded");
        jointPositionServer.setSucceeded(result);
    }
}

void Driver::cartesianPositionCallback(const ipa325_msgs::RobotMovementGoalConstPtr &goal)
{
    ROS_INFO_NAMED("driver", "received cartesian position");

    mutexRobotMovement.lock();

    ipa325_msgs::RobotMovementResult result;
    ipa325_msgs::RobotMovementFeedback feedback;

    //prepare feedback
    feedback.actual.pose.resize(6);
    feedback.desired.pose.resize(6);
    feedback.error.pose.resize(6);

    //create a command for all points
    for (int i = 0; i < goal->points.size(); i++)
    {
        //point data
        CommandCartesianPosition::MovementType movementType = (CommandCartesianPosition::MovementType)goal->points[i].movement_type;
        std::string targetFrameName = goal->points[i].frame_id;
        double blendRadius = goal->points[i].blend_radius;
        double velocity = goal->points[i].linVel;
        double acceleration = goal->points[i].linAcc;

        //verify point size
        if (goal->points[i].pose.size() != 6)
        {
            ROS_WARN_NAMED("driver", "positions count is %i but should be 6 (x, y, z, roll, pitch, yaw). This point will be ignored", (int)goal->points[i].pose.size());

            continue;
        }

        //movement type
        if (movementType == CommandCartesianPosition::PTP)
        {
            ROS_INFO_NAMED("driver", "PTP movement type");
        }
        else if (goal->points[i].movement_type == CommandCartesianPosition::LIN)
        {
            ROS_INFO_NAMED("driver", "LIN movement type");
        }
        else
        {
            ROS_WARN_NAMED("driver", "unknown movement type. This point will be ignored");

            continue;
        }

        //min blend radius
        if (blendRadius < 0.01)
        {
            ROS_WARN("blend radius %f is to tight. Setting default blend radius to 0.01", blendRadius);
            blendRadius = 0.01;
        }

        //limit velocity
        if (velocity > configuration.maxLinearVelocity)
        {
            ROS_WARN_NAMED("driver", "linear velocity limit reached. Limit linear velocity to %f", configuration.maxLinearVelocity);
            velocity = configuration.maxLinearVelocity;
        }

        //copy position
        CartesianPosition cartesianPosition;
        cartesianPosition.x() = goal->points[i].pose[0];
        cartesianPosition.y() = goal->points[i].pose[1];
        cartesianPosition.z() = goal->points[i].pose[2];
        cartesianPosition.rx() = goal->points[i].pose[3];
        cartesianPosition.ry() = goal->points[i].pose[4];
        cartesianPosition.rz() = goal->points[i].pose[5];

        //transform position from target frame into base frame
        if (targetFrameName != "")
        {
            try
            {
                ROS_INFO_NAMED("driver", "transform cartesian position into frame \"%s\"", targetFrameName.c_str());

                geometry_msgs::PoseStamped poseTarget;
                poseTarget.header.frame_id = targetFrameName;
                poseTarget.pose.position.x = cartesianPosition.x();
                poseTarget.pose.position.y = cartesianPosition.y();
                poseTarget.pose.position.z = cartesianPosition.z();
                tf::Vector3 rot = rpyToQuaternion(cartesianPosition.rx(), cartesianPosition.ry(), cartesianPosition.rz());
                poseTarget.pose.orientation.x = rot.x();
                poseTarget.pose.orientation.y = rot.y();
                poseTarget.pose.orientation.z = rot.z();
                poseTarget.pose.orientation.w = rot.w();

                geometry_msgs::PoseStamped poseBase;
                tfListener.transformPose(configuration.robotBaseFrameName, poseTarget, poseBase);

                cartesianPosition.x() = poseBase.pose.position.x;
                cartesianPosition.y() = poseBase.pose.position.y;
                cartesianPosition.z() = poseBase.pose.position.z;
                rot = quaternionToRpy(poseBase.pose.orientation.x, poseBase.pose.orientation.y, poseBase.pose.orientation.z, poseBase.pose.orientation.w);
                cartesianPosition.rx() = rot.x();
                cartesianPosition.ry() = rot.y();
                cartesianPosition.rz() = rot.z();
            }
            catch (tf::TransformException& e)
            {
                ROS_WARN_NAMED("driver", "transformation of position into base frame failed. This point will be ignored: %s", e.what());

                continue;
            }
        }

        //transformation for final position
        //Because the final position for the robot must be the position of the flange, but the input position is the
        //TCP position, consider the offset from flange to TCP
        try
        {
            tf::Pose poseFinal;
            poseFinal.setOrigin(tf::Vector3(cartesianPosition.x(), cartesianPosition.y(), cartesianPosition.z()));
            tf::Vector3 rot = rpyToQuaternion(cartesianPosition.rx(), cartesianPosition.ry(), cartesianPosition.rz());
            poseFinal.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));

            tf::StampedTransform transformFlange2Tcp;
            tfListener.lookupTransform(configuration.robotTcpFrameName, configuration.robotFlangeFrameName, ros::Time(0), transformFlange2Tcp);

            poseFinal *= transformFlange2Tcp;

            //final position
            cartesianPosition.x() = poseFinal.getOrigin().x();
            cartesianPosition.y() = poseFinal.getOrigin().y();
            cartesianPosition.z() = poseFinal.getOrigin().z();
            rot = quaternionToAxis(poseFinal.getRotation().x(), poseFinal.getRotation().y(), poseFinal.getRotation().z(), poseFinal.getRotation().w());
            cartesianPosition.rx() = rot.x();
            cartesianPosition.ry() = rot.y();
            cartesianPosition.rz() = rot.z();
        }
        catch (tf::TransformException& e)
        {
            ROS_WARN_NAMED("driver", "transformation of position into flange frame failed. This point will be ignored: %s", e.what());

            continue;
        }

        ROS_INFO_NAMED("driver", "movement type: %i, velocity: %f, acceleration: %f, blend radius: %f, position: %s", movementType, velocity, acceleration, blendRadius, cartesianPosition.toString().c_str());

        //add command to the command queue
        Command* command = new CommandCartesianPosition(cartesianPosition, movementType, velocity, acceleration);
        connector.addCommand(command);

        //send feedback while moving to target position
        ros::Rate rate(10);
        while(ros::ok() && !cartesianPositionServer.isPreemptRequested() && !shutdownSignal && !stopCommandReceived)
        {
            //set feedback
            feedback.actual.pose[0] = lastRobotState.getCartesianPosition().x();
            feedback.actual.pose[1] = lastRobotState.getCartesianPosition().y();
            feedback.actual.pose[2] = lastRobotState.getCartesianPosition().z();
            tf::Vector3 rot = axisToRpy(lastRobotState.getCartesianPosition().rx(), lastRobotState.getCartesianPosition().ry(), lastRobotState.getCartesianPosition().rz());
            feedback.actual.pose[3] = rot.x();
            feedback.actual.pose[4] = rot.y();
            feedback.actual.pose[5] = rot.z();

            feedback.desired.pose[0] = cartesianPosition.x();
            feedback.desired.pose[1] = cartesianPosition.y();
            feedback.desired.pose[2] = cartesianPosition.z();
            rot = axisToRpy(cartesianPosition.rx(), cartesianPosition.ry(), cartesianPosition.rz());
            feedback.desired.pose[3] = rot.x();
            feedback.desired.pose[4] = rot.y();
            feedback.desired.pose[5] = rot.z();

            feedback.error.pose[0] = feedback.desired.pose[0] - feedback.actual.pose[0];
            feedback.error.pose[1] = feedback.desired.pose[1] - feedback.actual.pose[1];
            feedback.error.pose[2] = feedback.desired.pose[2] - feedback.actual.pose[2];
            feedback.error.pose[3] = feedback.desired.pose[3] - feedback.actual.pose[3];
            feedback.error.pose[4] = feedback.desired.pose[4] - feedback.actual.pose[4];
            feedback.error.pose[5] = feedback.desired.pose[5] - feedback.actual.pose[5];

            //check if target position is reached
            bool reached = fabs(feedback.error.pose[0]) < blendRadius && 
                           fabs(feedback.error.pose[1]) < blendRadius && 
                           fabs(feedback.error.pose[2]) < blendRadius && 
                           fabs(feedback.error.pose[3]) < configuration.angleTolerance && 
                           fabs(feedback.error.pose[4]) < configuration.angleTolerance && 
                           fabs(feedback.error.pose[5]) < configuration.angleTolerance;

            //reached target position
            if (reached || configuration.isDummy)
            {
                ROS_INFO_NAMED("driver", "reached cartesian target position: %s", cartesianPosition.toString().c_str());

                break;
            }

            cartesianPositionServer.publishFeedback(feedback);

            rate.sleep();
        }
    }

    mutexRobotMovement.unlock();

    if(stopCommandReceived)
    {
        stopCommandReceived = false;

        ROS_INFO_NAMED("driver", "RobotMovement: Aborted");
        cartesianPositionServer.setAborted(result);
    }
    else if (cartesianPositionServer.isPreemptRequested())
    {
        ROS_INFO_NAMED("driver", "RobotMovement: Preempted");
        cartesianPositionServer.setPreempted(result);
    }
    else
    {
        ROS_INFO_NAMED("driver", "RobotMovement: Succeeded");
        cartesianPositionServer.setSucceeded(result);
    }
}
*/

void Driver::executeDigIo(const ur_driver::DigIOGoalConstPtr &goal)
{
    ROS_INFO_NAMED("driver", "execute Dig IO action received");

    ur_driver::DigIOResult result;

    if (goal->readOnly){
        result.state = lastRobotState.get_IO(goal->ioNr);
    } else {
        Command* command = new CommandDigitalIO(goal->ioNr, (bool)goal->newState);
        connector.addCommand(command);
        result.state = (bool)goal->newState;
    }
    result.ioNr = goal->ioNr;
    digitalIOServer.setSucceeded(result);
}

void Driver::executeDigIoArray(const ur_driver::DigIOArrayGoalConstPtr &goal)
{
    ROS_INFO_NAMED("driver", "execute Dig IO Array action received");

    ur_driver::DigIOArrayResult result;

    if (goal->ioNr.size() != goal->newState.size()){
        digitalIOArrayServer.setAborted(result, "Size of ioNr and newState do not match!");
        return;
    }

    int how_many_writes = 0;
    for (int i=0; i < goal->ioNr.size(); i++){
        if (!goal->readOnly[i]) how_many_writes++;
    }

    Command commands [how_many_writes];

    int k = 0;
    for (int i=0; i< goal->ioNr.size(); i++)
    {

        if (goal->readOnly[i]){
            result.ioNr.push_back(goal->ioNr[i]);
            result.state.push_back(lastRobotState.get_IO(goal->ioNr[i]));
        } else {
            commands[k] = CommandDigitalIO(goal->ioNr[i], (bool) goal->newState[i]);
            result.ioNr.push_back(goal->ioNr[i]);
            result.state.push_back((bool)goal->newState[i]);
            k++;
        }
    }

    CommandMultiCommand * multi = new CommandMultiCommand(commands, how_many_writes);
    if (how_many_writes > 0) connector.addCommand(multi);

    digitalIOArrayServer.setSucceeded(result);
}



void Driver::robotStatePublishWorker()
{
    ros::Rate rate(configuration.publishStateFrequency);

    while (ros::ok() && runRobotStatePublishThread)
    {
        //publish joint state
        sensor_msgs::JointState jointState;
        jointState.header.stamp = ros::Time::now();
        jointState.name = configuration.jointNames;
        jointState.position = lastRobotState.getJointPosition().getValues();
        jointState.velocity = lastRobotState.getJointVelocity().getValues();
        jointState.effort.resize(jointState.position.size(), 0);                //TODO get effort values

        if (jointState.position.size() > 0 && jointState.velocity.size() > 0)
        {
            jointStatePublisher.publish(jointState);
        }

        //publish pose state
        geometry_msgs::Pose poseState;
        poseState.position.x = lastRobotState.getCartesianPosition().x();
        poseState.position.y = lastRobotState.getCartesianPosition().y();
        poseState.position.z = lastRobotState.getCartesianPosition().z();
        tf::Vector3 rot = axisToQuaternion(lastRobotState.getCartesianPosition().rx(), lastRobotState.getCartesianPosition().ry(), lastRobotState.getCartesianPosition().rz());
        poseState.orientation.x = rot.x();
        poseState.orientation.y = rot.y();
        poseState.orientation.z = rot.z();
        poseState.orientation.w = rot.w();

        poseStatePublisher.publish(poseState);

        // publish pose with euler intrinsic zyx
        robot_movement_interface::EulerFrame pose_xyzState;
        pose_xyzState.x = lastRobotState.getCartesianPosition().x();
        pose_xyzState.y = lastRobotState.getCartesianPosition().y();
        pose_xyzState.z = lastRobotState.getCartesianPosition().z();
        tf::Vector3 rot2 = axisToRpy(lastRobotState.getCartesianPosition().rx(), lastRobotState.getCartesianPosition().ry(), lastRobotState.getCartesianPosition().rz());
        // axisToRPY produces extrinsic x,y,z -> we need intrinsic z,y,x (direct conversion by changing order)
        pose_xyzState.alpha = rot2.z();
        pose_xyzState.beta  = rot2.y();
        pose_xyzState.gamma = rot2.x();
        toolFrameStatePublisher.publish(pose_xyzState);

        //publish a frame for the position of the TCP. Because the given pose state is the position of the flange a transformation into the tool frame is necessary.
        try
        {
            tf::Pose tfPose;
            tf::poseMsgToTF(poseState, tfPose);
            tf::StampedTransform transformFlange2Tcp;
            tfListener.waitForTransform(configuration.robotFlangeFrameName, configuration.robotTcpFrameName, ros::Time(0), ros::Duration(0.5));
            tfListener.lookupTransform(configuration.robotFlangeFrameName, configuration.robotTcpFrameName, ros::Time(0), transformFlange2Tcp);
            tfPose *= transformFlange2Tcp;
            tfBroadcaster.sendTransform(tf::StampedTransform(tfPose, ros::Time::now(), configuration.robotBaseFrameName, "robot_state_tcp"));
        }
        catch (tf::TransformException& e)
        {
            ROS_ERROR_NAMED("driver", "TCP pose transformation failed: %s", e.what());
        }

        rate.sleep();
    }
}

void Driver::robotStateListener(const RobotState& robotState)
{
    //TODO need mutex? (probably yes, but no side effects so far)
    lastRobotState = robotState;
}

void Driver::signalHandler(int signal)
{
    ROS_INFO_NAMED("driver", "shutdown");
    shutdownSignal = true;
}

void Driver::commandThreadWorker()
{
    ros::Rate rate(configuration.robotReadFrequency);

    int result;

    while (ros::ok())
    {

        commandMutex.lock();

		if (commandList.size() > 0){
			if (isCommandFinished(commandList[0], &result)){

				isLastCommand = true;
				lastCommand = commandList[0];

				if (commandList[0].command_id >= 0){

					robot_movement_interface::Result result_msg;
		            result_msg.command_id = commandList[0].command_id;
		            result_msg.result_code = result;
		            commandResultPublisher.publish(result_msg); 

				}

				commandList.erase(commandList.begin());
			}
		}

        commandMutex.unlock();

        rate.sleep();
    }
}

bool Driver::isCommandFinished(robot_movement_interface::Command command, int *result){
    
    *result = 0;

    if (strcmp(command.pose_type.c_str(), "EULER_INTRINSIC_ZYX") == 0){

        // delta is the launch distance previous to blending, if not given then it should be low value but not 0 (over robot resolution)
        double dx = fabs(lastRobotState.getCartesianPosition().x() - command.pose[0]);
        double dy = fabs(lastRobotState.getCartesianPosition().y() - command.pose[1]);
        double dz = fabs(lastRobotState.getCartesianPosition().z() - command.pose[2]);

		float blending = 0.0;	// m
		float delta = 0.001;	// m

		if (command.blending.size() > 0) blending = command.blending[0];
		if (command.blending.size() > 1) delta = command.blending[1];

        return (dx * dx + dy * dy + dz * dz) <= (blending + delta) * (blending + delta);

    } else if (strcmp(command.pose_type.c_str(), "JOINTS") == 0){

		double sum = 0;
		double delta = 0.01; // 0.01 rad to consider a position correct

		if (command.blending.size() > 1) delta = command.blending[1];

		JointPosition pos =	lastRobotState.getJointPosition();	
		for (int i = 0; i < 6; i++) sum += fabs(command.pose[i] - pos[i]) * fabs(command.pose[i] - pos[i]);
		return sum <= (delta) * (delta); 
	}

    return false;

}

int Driver::processCommand(robot_movement_interface::Command command, ur_driver::Command * result){

	// ------------------------------------------------------------------------
	// Format validation
	// ------------------------------------------------------------------------

	if (strcmp(command.pose_type.c_str(), "JOINTS") == 0){
		if (command.pose.size() < 6) return 0;
	}

	if (strcmp(command.pose_type.c_str(), "EULER_INTRINSIC_ZYX") == 0){
		if (command.pose.size() < 6) return 0;
	}

	if (strcmp(command.velocity_type.c_str(), "M/S") == 0){
		if (command.velocity.size() < 1) return 0;
	}

	if (strcmp(command.acceleration_type.c_str(), "M/S^2") == 0){
		if (command.acceleration.size() < 1) return 0;
	}

	if (strcmp(command.velocity_type.c_str(), "RAD/S") == 0){
		if (command.velocity.size() < 1) return 0;
	}

	if (strcmp(command.acceleration_type.c_str(), "RAD/S^2") == 0){
		if (command.acceleration.size() < 1) return 0;
	}

	if (strcmp(command.blending_type.c_str(), "M") == 0){
		if (command.blending.size() < 1) return 0;
	}
	// ------------------------------------------------------------------------


	if (strcmp(command.command_type.c_str(), "LIN") == 0) {

		if (strcmp(command.pose_type.c_str(), "JOINTS") == 0) {

			JointValue position = JointValue(6);
			for (int i=0; i<6; i++) position[i] = command.pose[i];

			if (strcmp(command.velocity_type.c_str(), "M/S") != 0) return 0;
			if (strcmp(command.acceleration_type.c_str(), "M/S^2") != 0) return 0;
			if (strcmp(command.blending_type.c_str(), "M") != 0) return 0;

			*result = CommandLinJointBlending(position, command.velocity[0], command.acceleration[0], command.blending[0]);

		} else if (strcmp(command.pose_type.c_str(), "EULER_INTRINSIC_ZYX") == 0) {

			if (strcmp(command.velocity_type.c_str(), "M/S") != 0) return 0;
			if (strcmp(command.acceleration_type.c_str(), "M/S^2") != 0) return 0;
			if (strcmp(command.blending_type.c_str(), "M") != 0) return 0;

			// Euler intrinsic ZYX -> RPY extrinsic XYZ needs only to change order
			CartesianPosition cartesianPosition;
		    cartesianPosition.x() = command.pose[0];
		    cartesianPosition.y() = command.pose[1];
		    cartesianPosition.z() = command.pose[2];
			tf::Vector3 quaternion = rpyToQuaternion(command.pose[5], command.pose[4], command.pose[3]);
		    tf::Vector3 axis = quaternionToAxis(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
		    cartesianPosition.rx() = axis.x();
		    cartesianPosition.ry() = axis.y();
		    cartesianPosition.rz() = axis.z();

			*result = CommandLinCartesianBlending(cartesianPosition, command.velocity[0], command.acceleration[0], command.blending[0]);

		}

	} else if (strcmp(command.command_type.c_str(), "LIN_TIMED") == 0){

		if (strcmp(command.pose_type.c_str(), "JOINTS") != 0) return 0;
	
        JointPosition jointPosition(6);
		for (int i = 0; i < 6; i++) jointPosition[i] = command.pose[i];

		double velocity = configuration.velocity; // No matters, time has priority
        double acceleration = configuration.acceleration; // No matters, time has priority

		if (strcmp(command.blending_type.c_str(), "M") != 0) return 0;
		double blending = command.blending[0];

		if (command.additional_values.size() < 1) return 0;
		double time = command.additional_values[0];

        *result = CommandLinJointTimed(jointPosition, velocity, acceleration, blending, time);

	} else if (strcmp(command.command_type.c_str(), "PTP") == 0) {

		if (strcmp(command.pose_type.c_str(), "JOINTS") == 0) {

			JointValue position = JointValue(6);
			for (int i=0; i<6; i++) position[i] = command.pose[i];

			if (strcmp(command.velocity_type.c_str(), "RAD/S") != 0) return 0;
			if (strcmp(command.acceleration_type.c_str(), "RAD/S^2") != 0) return 0;
			if (strcmp(command.blending_type.c_str(), "M") != 0) return 0;

			*result = CommandPtpJointBlending(position, command.velocity[0], command.acceleration[0], command.blending[0]);

		} else if (strcmp(command.pose_type.c_str(), "EULER_INTRINSIC_ZYX") == 0) {

			if (strcmp(command.velocity_type.c_str(), "RAD/S") != 0) return 0;
			if (strcmp(command.acceleration_type.c_str(), "RAD/S^2") != 0) return 0;
			if (strcmp(command.blending_type.c_str(), "M") != 0) return 0;

			// Euler intrinsic ZYX -> RPY extrinsic XYZ needs only to change order
			CartesianPosition cartesianPosition;
		    cartesianPosition.x() = command.pose[0];
		    cartesianPosition.y() = command.pose[1];
		    cartesianPosition.z() = command.pose[2];
			tf::Vector3 quaternion = rpyToQuaternion(command.pose[5], command.pose[4], command.pose[3]);
		    tf::Vector3 axis = quaternionToAxis(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
		    cartesianPosition.rx() = axis.x();
		    cartesianPosition.ry() = axis.y();
		    cartesianPosition.rz() = axis.z();

			*result = CommandPtpCartesianBlending(cartesianPosition, command.velocity[0], command.acceleration[0], command.blending[0]);

		}

	} else if (strcmp(command.command_type.c_str(), "JOINT_SPEED") == 0){


		if (command.velocity.size() < 6) return 0;

        JointVelocity jointVelocity(6);
        for (int i=0; i<6; i++) jointVelocity[i] = command.velocity[i];

		if (strcmp(command.acceleration_type.c_str(), "RAD/S^2") != 0) return 0;

		double acceleration = command.acceleration[0];

		if (command.additional_values.size() == 0){ 
			*result = CommandJointVelocity(jointVelocity, acceleration, 1.0);
		} else {
			*result = CommandJointVelocity(jointVelocity, acceleration, command.additional_values[0]);
		}

	} else if (strcmp(command.command_type.c_str(), "CARTESIAN_SPEED") == 0){

		if (command.velocity.size() < 6) return 0;
        
        CartesianVelocity cartesianVelocity;
        cartesianVelocity.x() = command.velocity[0];
        cartesianVelocity.y() = command.velocity[1];
        cartesianVelocity.z() = command.velocity[2];
        // Euler intrinsic ZYX -> RPY extrinsic XYZ needs only to change order
        tf::Vector3 quaternion = rpyToQuaternion(command.velocity[5], command.velocity[4], command.velocity[3]);
        tf::Vector3 axis = quaternionToAxis(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        cartesianVelocity.rx() = axis.x();
        cartesianVelocity.ry() = axis.y();
        cartesianVelocity.rz() = axis.z();

		double acceleration = configuration.acceleration;
		if (strcmp(command.acceleration_type.c_str(), "M/S^2") == 0) acceleration = command.acceleration[0];

		if (command.additional_values.size() == 0){ 
			*result = CommandCartesianVelocity(cartesianVelocity, acceleration, 1.0);
		} else {
			*result = CommandCartesianVelocity(cartesianVelocity, acceleration, command.additional_values[0]);
		}

	} else {
		return 0;
	}

	return 1;

}


void Driver::commandListCallback(const robot_movement_interface::CommandListConstPtr &msg)
{

	if (!msg->replace_previous_commands) return; // No more addition allowed, can be simulated by repeating commands in the new trajectory
    
    commandMutex.lock();

	if (msg->replace_previous_commands) commandList.clear(); // Deletion in cascade should prevent from memory leaks

	for (int i = 0; i < msg->commands.size(); i++) commandList.push_back(msg->commands[i]); // Commands are copied including copy in cascade of the vectors
	
	// Replace quaternions with euler coordinates
	replaceQuaternions(commandList);

	if (msg->commands.size() > 0){
		Command commands [commandList.size()];
		bool differential_found = false;
		for (int i = 0; i < commandList.size(); i++){
			if (processCommand(commandList[i], &commands[i]) == 0){			
				std::cerr << "Error in command list, aborting...";
				commandList.clear();
				commandMutex.unlock();
				return;
			}
			if (strcmp(commandList[i].command_type.c_str(), "JOINT_SPEED") == 0) differential_found = true; // The commands include a differential command, no result will be provided
			if (strcmp(commandList[i].command_type.c_str(), "CARTESIAN_SPEED") == 0) differential_found = true; // The commands include a differential command, no result will be provided
		}

		CommandMultiCommand * multi = new CommandMultiCommand(commands, commandList.size());

		if (differential_found){
			commandList.clear();
			isLastCommand = false;
		}

		connector.addCommand(multi);
	} else {
		// Send stop command if replace == true
		if (msg->replace_previous_commands){
			Command * stopcommand = new CommandStop(configuration.acceleration);
			connector.addCommand(stopcommand);
		}
	}

    commandMutex.unlock();

    //ROS_INFO_NAMED("driver", "executed command list");

}

// Replaces all quaternions with Euler coordinates
void Driver::replaceQuaternions(std::vector<robot_movement_interface::Command> & list){
	for (int i = 0; i < list.size(); i++){
		if (strcmp(list[i].pose_type.c_str(), "QUATERNION") == 0){
			list[i].pose_type = "EULER_INTRINSIC_ZYX";
			list[i].pose = transformQuaternionToEulerIntrinsicZYX(list[i].pose);
		}
	}
}

// Transforms a quaternion to Euler Intrinsic ZYX through Rotation Matrix
std::vector<float> Driver::transformQuaternionToEulerIntrinsicZYX(std::vector<float> frame_quaternion){

	assert(frame_quaternion.size() == 7); // Test correct input
	
	float z,y,x;
	
	transformQuaternionToEulerIntrinsicZYX(frame_quaternion[3], frame_quaternion[4], frame_quaternion[5], frame_quaternion[6], &z, &y, &x );
	
	std::vector<float> euler;
	for (int i=0; i<3; i++) euler.push_back(frame_quaternion[i]);
	euler.push_back(z);
	euler.push_back(y);
	euler.push_back(x);
	return euler;
}

// Transforms a quaternion to Euler Intrinsic ZYX through Rotation Matrix
void Driver::transformQuaternionToEulerIntrinsicZYX(float qx, float qy, float qz, float qw, float * z, float * y, float * x ){

	float m[3][3];
	
	m[0][0] = 1 - 2*qy*qy - 2*qz*qz;
    m[0][1] = 2*qx*qy - 2*qz*qw;
    m[0][2] = 2*qx*qz + 2*qy*qw;
    m[1][0] = 2*qx*qy + 2*qz*qw;
    m[1][1] = 1 - 2*qx*qx - 2*qz*qz;
    m[1][2] = 2*qy*qz - 2*qx*qw;
    m[2][0] = 2*qx*qz - 2*qy*qw;
    m[2][1] = 2*qy*qz + 2*qx*qw;
    m[2][2] = 1 - 2*qx*qx - 2*qy*qy;

	*z = atan2( m[1][0], m[0][0]);
    *y = atan2(-m[2][0], sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]));
    *x = atan2( m[2][1], m[2][2]);
	
}
