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

#include <command.h>

using namespace ur_driver;

//=================================================================
// Commands
//=================================================================

std::string Command::getCommandString()
{
    return commandString;
}

CommandJointPosition::CommandJointPosition(JointValue position, double speed, double accel)
{
    char buffer[255];

    snprintf(buffer, 255, "movej([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f)\n",
        position[0],
        position[1],
        position[2],
        position[3],
        position[4],
        position[5],
        accel,
        speed);

    commandString = std::string(buffer);
}

CommandJointTimedPosition::CommandJointTimedPosition(JointValue position, double speed, double accel, double blending, double time)
{
    char buffer[255];

    snprintf(buffer, 255, "movej([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f, %5.5f, %5.5f)\n",
        position[0],
        position[1],
        position[2],
        position[3],
        position[4],
        position[5],
        accel,
        speed,
		time,
		blending);

    commandString = std::string(buffer);
}

CommandJointStop::CommandJointStop(double accel)
{
    char buffer[255];

    snprintf(buffer, 255, "stopj(%5.5f)\n", accel);

    commandString = std::string(buffer);
}

CommandCartesianPosition::CommandCartesianPosition(CartesianValue position, MovementType movementType, double speed, double accel)
{
    char buffer[255];

    if (movementType == LIN)
    {
        snprintf(buffer, 255, "movel(p[%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f)\n",
            position.x(),
            position.y(),
            position.z(),
            position.rx(),
            position.ry(),
            position.rz(),
            speed,
            accel);
    }
    else if (movementType == PTP)
    {
        snprintf(buffer, 255, "movej(p[%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f)\n",
            position.x(),
            position.y(),
            position.z(),
            position.rx(),
            position.ry(),
            position.rz(),
            speed,
            accel);
    }

    commandString = std::string(buffer);
}

CommandCartesianStop::CommandCartesianStop(double accel)
{
    char buffer[255];

    snprintf(buffer, 255, "stopl(%5.5f)\n", accel);

    commandString = std::string(buffer);
}

CommandDigitalIO::CommandDigitalIO(int id, bool value)
{
    char buffer[255];

    // 0-7 digital input, 8-15 configurable input, 16-17 tool input, 18-25 digital output, 26-33 configurable output, 34-35 tool output
    if ((id >= 18) && (id < 26)) snprintf(buffer, 255, "set_digital_out(%d, %s)\n", id - 18, (value) ? "True" : "False");
    if ((id >= 26) && (id < 34)) snprintf(buffer, 255, "set_configurable_digital_out(%d, %s)\n", id - 26, (value) ? "True" : "False");
    if ((id >= 34) && (id < 36)) snprintf(buffer, 255, "set_tool_digital_out(%d, %s)\n", id - 34, (value) ? "True" : "False");

    printf("%s", buffer);

    commandString = std::string(buffer);
}

CommandAnalogIO::CommandAnalogIO(int id, double value)
{
    char buffer[255];

    snprintf(buffer, 255, "set_analog_out(%d, %5.5f)\n", id, value);

    commandString = std::string(buffer);
}
/*
CommandCartesianPositionBlending::CommandCartesianPositionBlending(CartesianValue position, MovementType movementType, double speed, double accel, double blending)
{
    char buffer[255];

    // movel(pose, a=1.2, v=0.3, t=0, r=0)
    // movep(pose, a=1.2, v=0.3, r=0)

    if (movementType == LIN)
    {
        snprintf(buffer, 255, "movep(p[%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], a=%5.5f, v=%5.5f, r=%5.5f)\n",
            position.x(),
            position.y(),
            position.z(),
            position.rx(),
            position.ry(),
            position.rz(),
            accel,
            speed,
	        blending);
    }

    commandString = std::string(buffer);
}*/

// --------------------------------------------------------------------
// Euroc
// --------------------------------------------------------------------

CommandMultiCommand::CommandMultiCommand(Command * commands, int size){

	commandString.clear();

	commandString.append("def multi():\r\n");

	for (int i=0; i < size; i++){
		commandString.append("  ");
		commandString.append(commands[i].getCommandString());
	}

	commandString.append("end\r\nmulti()\r\n");

}

CommandStop::CommandStop(double acceleration)
{
    char buffer[255];
   	snprintf(buffer, 255, "stopj(a=%5.5f)\n", acceleration);
	commandString = std::string(buffer);
}

CommandCartesianVelocity::CommandCartesianVelocity(CartesianVelocity velocity, double accel, double time)
{
    char buffer[255];

    snprintf(buffer, 255, "speedl([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f)\n",
        velocity.x(),
        velocity.y(),
        velocity.z(),
        velocity.rx(),
        velocity.ry(),
        velocity.rz(),
        accel,
        time);

    commandString = std::string(buffer);
}

CommandJointVelocity::CommandJointVelocity(JointVelocity velocity, double accel, double time)
{
    char buffer[255];

    snprintf(buffer, 255, "speedj([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f)\n",
    velocity[0],
    velocity[1],
    velocity[2],
    velocity[3],
    velocity[4],
    velocity[5],
    accel,
    time);

    commandString = std::string(buffer);
}

CommandLinCartesianBlending::CommandLinCartesianBlending(CartesianValue position, double speed, double accel, double blending){
    char buffer[255];
    snprintf(buffer, 255, "movep(p[%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], a=%5.5f, v=%5.5f, r=%5.5f)\n",
        position.x(),
        position.y(),
        position.z(),
        position.rx(),
        position.ry(),
        position.rz(),
        accel,
        speed,
        blending);
    commandString = std::string(buffer);
}

CommandPtpCartesianBlending::CommandPtpCartesianBlending(CartesianValue position, double speed, double accel, double blending){
    char buffer[255];
    snprintf(buffer, 255, "movej(p[%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], a=%5.5f, v=%5.5f, r=%5.5f)\n",
        position.x(),
        position.y(),
        position.z(),
        position.rx(),
        position.ry(),
        position.rz(),
        accel,
        speed,
        blending);
    commandString = std::string(buffer);
}

CommandLinJointBlending::CommandLinJointBlending(JointValue position, double speed, double accel, double blending){
    char buffer[255];
    snprintf(buffer, 255, "movep([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], a=%5.5f, v=%5.5f, r=%5.5f)\n",
        position[0],
        position[1],
        position[2],
        position[3],
        position[4],
        position[5],
        accel,
        speed,
        blending);
    commandString = std::string(buffer);
}

CommandPtpJointBlending::CommandPtpJointBlending(JointValue position, double speed, double accel, double blending){
    char buffer[255];
    snprintf(buffer, 255, "movej([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], a=%5.5f, v=%5.5f, r=%5.5f)\n",
        position[0],
        position[1],
        position[2],
        position[3],
        position[4],
        position[5],
        accel,
        speed,
        blending);
    commandString = std::string(buffer);
}

CommandLinJointTimed::CommandLinJointTimed(JointValue position, double speed, double accel, double blending, double time)
{
    char buffer[255];

    snprintf(buffer, 255, "movel([%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], %5.5f, %5.5f, %5.5f, %5.5f)\n",
        position[0],
        position[1],
        position[2],
        position[3],
        position[4],
        position[5],
        accel,
        speed,
		time,
		blending);

    commandString = std::string(buffer);
}
