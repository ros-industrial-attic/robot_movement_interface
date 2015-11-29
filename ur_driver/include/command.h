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
// Base class for commands which can be send to the robot. 
// A command is a string which represents a simple script to execute on the robot controller.
// The commands will be send to the robot controller through the Controller class. 
// Use Controller::addCommand to add commands to the output queue.
// ----------------------------------------------------------------------------

#ifndef COMMAND_H_
#define COMMAND_H_

#include <utils.h>

namespace ur_driver
{
    //=================================================================
    // Commands
    //=================================================================
    /**
     * Base class for commands which can be send to the robot. A command is a string which represents a simple script to execute on the robot controller.
     * The commands will be send to the robot controller through the Controller class. Use Controller::addCommand to add commands to the output queue.
     */
    class Command
    {
        protected:
            std::string commandString;

        public:
            std::string getCommandString();
    };

    class CommandJointPosition : public Command
    {
        public:
            CommandJointPosition(JointValue position, double speed, double accel);
    };

	class CommandJointTimedPosition : public Command
    {
        public:
            CommandJointTimedPosition(JointValue position, double speed, double accel, double blending, double time);
    };

    class CommandJointStop : public Command
    {
        public:
            CommandJointStop(double accel);
    };

    class CommandCartesianPosition : public Command
    {
        public:
            typedef enum MovementType
            {
                PTP = 0,
                LIN = 1
            } MovementType;

            CommandCartesianPosition(CartesianValue position, MovementType movementType, double speed, double accel);
    };

    class CommandCartesianStop : public Command
    {
        public:
            CommandCartesianStop(double accel);
    };

    class CommandDigitalIO : public Command
    {
        public:
            CommandDigitalIO(int id, bool value);
    };

    class CommandAnalogIO : public Command
    {
        public:
            CommandAnalogIO(int id, double value);
    };

	/*class CommandCartesianPositionBlending : public Command
    {
        public:
            typedef enum MovementType
            {
                PTP = 0,
                LIN = 1
            } MovementType;

            CommandCartesianPositionBlending(CartesianValue position, MovementType movementType, double speed, double accel, double blending);
    };*/

	// --------------------------------------------------------------------
	// Euroc
	// --------------------------------------------------------------------

	class CommandMultiCommand : public Command
	{
		public:
			CommandMultiCommand(Command * commands, int size);
	};

	// Stop command
	class CommandStop : public Command
	{
		public:
			CommandStop(double acceleration);
	};

	class CommandCartesianVelocity : public Command
    {
        public:
            CommandCartesianVelocity(CartesianVelocity velocity, double accel, double time);
    };

	class CommandJointVelocity : public Command
    {
        public:
            CommandJointVelocity(JointVelocity velocity, double accel, double time);
    };

	class CommandLinJointTimed : public Command
    {
        public:
            CommandLinJointTimed(JointValue position, double speed, double accel, double blending, double time);
    };

	class CommandLinCartesianBlending : public Command
	{
		public:
			CommandLinCartesianBlending(CartesianValue position, double speed, double accel, double blending);
	};

	class CommandPtpCartesianBlending : public Command
	{
		public:
			CommandPtpCartesianBlending(CartesianValue position, double speed, double accel, double blending);
	};

	class CommandLinJointBlending : public Command
	{
		public:
			CommandLinJointBlending(JointValue position, double speed, double accel, double blending);
	};

	class CommandPtpJointBlending : public Command
	{
		public:
			CommandPtpJointBlending(JointValue position, double speed, double accel, double blending);
	};
}


#endif
