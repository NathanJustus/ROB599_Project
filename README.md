# ROB599_Project

Nathan Justus
December 9th, Year of Our Lord 2020

This is my final project for the ROB599 ROS Development class.  You can see a description of my original pitch in the 'ROB599 Project Pitch' pdf file included.

Launch the three main nodes using
`rosrun rob599_project flapper.launch`

You can use the service to set goal locations by 
`rosservice call /set_swim_goal <x_goal> <y_goal> <angle_goal> <do_goal_boolean>`

`<x_goal>`, `<y_goal>`, and `<angle_goal>` are all positive floats that set the goal pose of the robot with `0 <= x/y_goal <= 6` and `0 <= angle_goal <= 2*pi`.  I never actually implemented control for the angle_goal, so all it does is change the goal in the animation.  The `<do_goal_boolean>` should be either 0 or 1, 0 clears old goals and makes the robot move forward indefinitely, and 1 tells the controller to actually try to navigate to this goal.

Implemented in ROS Melodic
