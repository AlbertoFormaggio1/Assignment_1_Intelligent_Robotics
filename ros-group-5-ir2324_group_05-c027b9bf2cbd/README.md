The files for assignment 1 are inside the tiago_action package.

To launch the package use

roslaunch tiago_action tiago_action.launch point_b:="x y w"

A working example (used also in the video):
roslaunch tiago_action tiago_action.launch point_b:="10.8 -0.2 -0.4"

<br>------------------------------------------------<br>

From this position, the coordinates in the Gazebo reference frame found by the robot (**prediction**) are:

P = {(5.426, 0.809), (5.820, -0.988), (4.544, -2.088), (3.969, -0.057)}
<br><br><br>
while the value we get from the pose in Gazebo (**true values**) are:

T = {(5.811, 0.763), (6.160, -1.347), (4.875, -2.424), (4.155, -0.295)}

