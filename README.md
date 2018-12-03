# RBE 3002 Lab Group 02
Jonathan Berry, Erika Snow, and Benjamin Wagner


## Setup Instructions

Before running the code on a new computer:
- clone the repository into the `src` directory of a catkin workspace and run catkin_make
- make sure the commands `source /opt/ros/kinetic/setup.bash`, `source ~/catkin_ws/devel/setup.bash`, and `export TURTLEBOT3_MODEL="burger"` are included as separate lines in `~/.bashrc`
- copy the contents of `models` (the `Maze` and `Maze2` directories) into `~/.gazebo/models`


## Operation Instructions

To run the code, run:

`roslaunch rbe3002code02 rviz.launch`

Then, in rviz, select an end point within the known empty space with the "2D Nav Goal" tool. The robot will attempt to navigate to that location.
