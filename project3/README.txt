1. Running my assignment:
Ternimal 1: 
roslaunch project3 baxter_project3.launch
wait until you see:
[ INFO] [1494644646.618441110, 34.956000000]: Simulator is loaded and started successfully
[ INFO] [1494644646.626027480, 34.964000000]: Robot is disabled
[ INFO] [1494644646.626223664, 34.964000000]: Gravity compensation was turned off
This is when gazebo loaded completely.
Load the rviz config, you might want to add two more tf frames to see source and destinations for the chess piece

Terminal 2: 
rosrun baxter_tools enable_robot.py -e (Running this twice doesn't hurt - sometimes the robot is not enabled)
rosrun baxter_interface joint_trajectory_action_server.py 

Ternimal 3:
roslaunch project3 my_baxter.launch

Ternimal 4:
rosrun project3 tf_fix.py (To run my tf for pick and place)
You can see it priting (a1, a1) on screen this is the default pick and place locations.

Ternimal 5:
syntax: rostopic pub -1 /pick_and_place mv_msgs/move_to_from '{header: auto, move_from: <source>, move_to: <destination>}'
example: rostopic pub -1 /pick_and_place mv_msgs/move_to_from '{header: auto, move_from: a3, move_to: h5}'
This will command the robot to pick an imaginary chess piece from a3 and place it at h5.

Terminal 6: 
rosrun project3 project3.py joint_states:=/robot/joint_states

------------------------------------------------------------------------------------------------------
2. Issues solved/unsolved:
a. Can't reach all the cells (a1, b1, c1) - Unsolved
We might want to move the chessboard a little in front of the robot. (I remember from class you won't be testing those cells)
b. Error: LBKPIECE1: Motion planning start tree could not be initialized! Even for a valid end effector pose - solved
solution, run the node as follows:
rosrun project3 project3.py joint_states:=/robot/joint_states

------------------------------------------------------------------------------------------------------
3. Assumptions:
a.  I assumed that you would test the project by publishing a topic of message type specified the problem statement. 

------------------------------------------------------------------------------------------------------
4. If nothing works:
a. double check to see if all the ternimals are sourced or not.
b. See if the robot is enabled or not.
