# This message contains a list of commands to be executed by the driver
# If add is true, then commands should be added, if false, previous not-executed commands are replaced

# Timestamp information
Header header

# New commands to be executed
Command[] commands

# Optional: Frame coordinates to execute the command
# if not set, command will be executed in '/base' frame
EulerFrame frame

# True  -> Replaces any planned command with current new commands
# False -> Adds the new commands to the command queue
bool replace_previous_commands
