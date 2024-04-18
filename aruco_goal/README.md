# Aruco Goal

This program subscribes to arcuo messages to gather it's transforms, it processes those transforms and sends it to move_base through the action client

## Notes

To test this code, 

1. open terminal
2. run roscore using: roscore
3. open another terminal
4. run rviz using: rosrun rviz rviz (we can use this to see the camera later)
5. open another terminal
6. cd into catkin_ws and run catkin_make at least once
7. source devel/setup.bash
8. run camera using : roslaunch zed_wrapper zed2i.launch
9. open another terminal
10. run aruco detect node using : roslaunch aruco_detect.launch (may need to copy your own file and change the parameters as it has camera, image, fiducial_len, and dictionary size)
11. open another terminal
12. run move_base by going into catkin_ws/src/auto/navigation_stack/launch and use : roslaunch move_base.launch
13. open the sixth and final terminal
14. go to catkin_ws/src/auto/aruco_goal/src and run transform.py using: python transform.py

tags should be detected by the code and be sent to move_base now

to visualize this go to the rviz window and 
1. click add -> by topic tab -> fiducial_images -> camera -> compressed and ok

## Possible Future Implementation

the code may be altered to give different goals, 
currently everything is in respect to the camera in terms of posistion and orientation.

currently it passes a goal to the rover from the sign in a line directly to it from the angle the camera sees the tag, and stops 1.5 meters before it reaches the sign. 
if the rover wants to be looking a certain direction once it reaches that goal in respect to the aruco tag, modification to the function get_orientation is needed.

it does not currently take into account rover rotation in comparison to the camera, so if the camera is offset or rotated the rover may not move as intended. 
