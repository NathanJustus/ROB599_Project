Nathan Justus
December 9th, Year of Our Lord 2020

This is my final project for the ROB599 ROS Development class.  You can see a description of my original pitch in the 'ROB599 Project Pitch' pdf file included.

Launch the three main nodes using
rosrun rob599_project flapper.launch

You can use the service to set goal locations by 
rosservice call /set_swim_goal <x_goal> <y_goal> <angle_goal> <do_goal_boolean>

x_goal, y_goal, and angle_goal are all floats that set the goal pose of the robot.  I never actually implemented control for the angle_goal, so it's kind of useless right now.  The do_goal_boolean should be either 0 or 1, 0 clears old goals and makes the robot move forward indefinitely, and 1 tells the controller to actually try to navigate to this goal.

Grading myself from the rubric I made: Total (51/60)
- Load connection field (5/5)
- Load joint gaits (0/5): just decided to make my own, decided importing optimal gaits would be too big a hassle for 5 points
- Visualize 5-Link robot swimmer somehow (13/15): 2 points off because I wanted to use Gazebo and still might regret not doing so in the future if I ever want collision checking.  Just didn't make sense to set up a physics engine on my robot only to override the physics with my connection calculations
- Commanding joint movement causes appropriate visualization change (10/10)
- Commanding joint movement causes spatial movement from connection (20/20)
- Rudimentary path planner controls robot (3/5): 2 points off because the controller isn't as effective as I hoped.  It still mostly works, and the reasons it doesn't are motivators for my PhD work, so I count it as a win.