# application_template
Template for a ROS2 Application within our teleop stack. 

## How to use

1. Check this Repository out into your workspace
2. Copy the whole folder structure into your new application folder
3. Replace all variations of "templateManager" with your application name
4. If it is a module on remote machine, change forkliftHeartbeat to remoteHeartbeat in subscribers, callbacks and variable names
5. Add more subscribers, publishers and variables as needed
6. Keep the functions in the manager class, the ROS interfaces in the node class
7. Add your main function in the manager class
8. Add @brief tags to all functions in the header file


