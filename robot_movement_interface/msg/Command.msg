# This is a flexible command message for robotic drivers

# Header includes time stamps
Header header

# Command id
# This id will be copied to the result message to identify the executed command
# Although it doesn't affect to the driver functionality, it should be unique to allow future association
uint32 command_id

# Command type contains the move type (LIN, PTP, JOINT_DIRECT, ...)
string command_type

# If applicable, indicates which frame is used as base frame (relative positions)
string pose_reference

string pose_type
float32[] pose

string velocity_type
float32[] velocity

string acceleration_type
float32[] acceleration

string force_threshold_type
float32[] force_threshold

string effort_type
float32[] effort

string blending_type
float32[] blending

# Reserved for future extension
string[] additional_parameters
float32[] additional_values
