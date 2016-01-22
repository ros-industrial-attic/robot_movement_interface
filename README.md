# robot_movement_interface
Drivers implementing the Robot Movement Interface, which allows access to robot specific capabilities in a standardized common format while maintaining compatibility with the ROS-Industrial Joint interface.

Please read included README files for detailed information.

This metapackage currently contains:
-	iiwa_driver -> ROS iiwa driver (Robot Movement Interface and ROS Industrial Interface with MoveIt action)
-	ur_driver -> ROS UR driver (Robot Movement Interface and MoveIt action, Control Box 3.x supported)
-	robot_movement_interface -> shared message package for the drivers
-	dependecies -> common dependencies for some drivers.

This is the initial version of the driver after refactoring. Progresively more features and drivers would be added.

Please report problems or suggestions to Pablo.Quilez.Velilla@ipa.fraunhofer.de

===============================================================================
Instructions
===============================================================================

Before you can pass commands to a robot, it is import to start the appropriate robot driver by typing one of the following commands:
- 	roslaunch ur_driver start.launch<br/>
	Type: UR5/UR10 driver
- 	roslaunch iiwa_driver start.launch<br/>
	Type: Kuka IIWA driver

Additionally we have to start the actionizer script for wrapping all commands which are sent to the robot into the approriate topic and for getting the status of the robot action e.g. succeeded, aborted etc.
-	rosrun robot_movement_interface actionizer.py

===============================================================================
Robot Movement Interface
===============================================================================

Robot Movement Interface provides a ROS interface which is used to transform human friendly commands to robot-specific commands, as well as to access robot functionalities such as for example blending or force controlled commands without requiring a parallel independent and specific interface.

User programs send motion commands and become feedback through ROS topics provided by the driver module. Driver can be a monolithic application or a modular ROS node architecture. Driver module processes user friendly standard formatted messages into robot-specific commands and sends them normally through TCP/IP.

The driver also publishes robot state information as independent topics. The information published is different for each robot according to the specific robot capabilities.

===============================================================================
Interface description
===============================================================================

Robot Movement Interface defines the following two topics:
-	/command_list: for robot controlling<br/>
	Type: robot_movement_interface/CommandList
-	/command_result: for robot feedback<br/>
	Type: robot_movement_interface/Result
	
Additionally, each driver provides robot state topics to publish the current robot position, joints or specific information. Every driver must publish as minimal state information the following topics:
-	/joint_states: joint angles publishing (in radians)<br/>
	Type: sensor_msgs/JointState
-	/tool_frame: tool frame in m and radians following Euler Intrinsic ZYX convention<br/>
	Type: robot_movement_interface/EulerFrame
Position in quaternions or force if available is also published as independent topics following ROS conventions.

===============================================================================
Message description
===============================================================================

Each topic only accepts messages formatted with the correct message type. This message types is defined as common dependency for every driver (ipa325_common ROS package).

Controlling topic accepts a list of commands and also a flag to indicate if the received commands must replace the current robot motion or execute after the termination of the previous ones. 

Depending on the robot functionality driver could only accept replacement or addition.

Feedback topic publishes the corresponding command id which it refers, error code and additional information.

Command_list.msg:

	This message contains a trajectory / command batch.

	- 	ipa325_robot_movement_v2_command[] commands -> individual commands
	-	bool replace_previous_commands -> If commands replaces current motion
	
	
Command.msg:

	A list of all possible commands is described in the table.
	
	-	uint32 command_id -> this id is used for feedback publishing
	-	string command_type -> defines the command type
	-	string pose_reference -> defines the base frame of the target pose
	-	string pose_type -> defines in which format is the position provided (joints, euler, quaternions…)
	-	float32[] pose -> pose values in standard units (m, radians)
	-	string velocity_type -> defines in which format is the velocity provided (rad/s, m/s, %...)
	-	float32[] velocity -> speed values
	-	string acceleration_type -> defines in which format is the acceleration provided (m/s^2, %...)
	-	float32[] acceleration -> acceleration values
	-	string force_threshold_type -> defines in which format the cartesian force thresholds are passed (Typical: N)
	-	float32 force_threshold -> cartesian force thresholds in x, y, z when robot moves force constrained
	-	string effort_type -> defines in which format is the robot effort provided
	-	float32[] effort -> effort values
	-	string blending_type -> blending units (m, rad, %...)
	-	float32[] blending -> blending values
	-	string[] additional_parameters -> additional required parameter description
	-	float32[] additional_values -> value of the additional parameter, for example time constraint
	
Result.msg

	Feedback produced after termination of each command is published with this format.
	
	-	command_id -> matching command which the result refers to.
	-	result_code -> error code, normally 0 for no error
	-	additional_information -> flexible output
	
EulerFrame.msg

	This message format is used to publish the robot tool position.
	-	float32 x -> Cartesian X value in meters
	-	float32 y -> Cartesian Y value in meters
	-	float32 z -> Cartesian Z value in meters
	-	float32 alpha -> Rotation in Z in radians according to the Euler Tait-Bryan [4] intrinsic convention
	-	float32 beta -> Rotation in Y in radians according to the Euler Tait-Bryan intrinsic convention
	-	float32 gamma -> Rotation in X in radians according to the Euler Tait-Bryan intrinsic convention
	
===============================================================================
Allowed commands
===============================================================================

This list can be also found as Excel table in robot_movement_command_message_format_table.ods document (in top directory)

Each driver provides a different set of commands according to the robot capabilities. Some of the command fields could be required, optional or ignored by the driver (green, orange and yellow colours).



