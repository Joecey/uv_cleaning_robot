# UV Cleaning Robot
Github respository for WeBots simulation of a UV Cleaning robot for completion of the "Introduction to Robotics" module at Trinity College Dublin

## How navigation works for UV cleaning robot
* Robot is controlled by "uv_robot_main" controller and given 2-D occupancy grid of room (SampleTestRoom1 used as example)
* After beginning program, robot locates nearest corner in room and navigates towards nearest waypoint. Robot stops when close to obstacles
* Robot begins cleaning by slowly moving to next waypoint (counter clockwise direction), avoiding any obstacles that are in path
* After reaching all four corners of room, robot navigates to the centre of the room where CT-scanner is
* Depending where robot starts (i.e. above or below CT scanner), robot undergoes specific cleaning protocal by following the edges of the CT scanner until it meets an obstacle (i.e. an edge perpendicular to the edge it is currently cleaning)
* One robot has completed cleaning CT scanner, automated cleaning stops and robot switches to manual cleaning mode
