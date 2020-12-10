#!/usr/bin/env python

# Node to make flapper swim through water, either to goal or just indefinitely straight

# Import the good stuff
import rospy
import sys
import tf
import numpy as np
from math import atan2, pi, sqrt

#Import custom message definitions
from rob599_project.srv import SetGoal, SetGoalResponse

#Import predefined message definitions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_msgs.msg import String, Int64

#Class definition that will run all the functionality of the service servers
class Controller:

	#Called when instance of class is initialized
	def __init__(self):

		#Publisher of joint angles for control
		self.joint_publisher = rospy.Publisher('joint_angles',Float32MultiArray,queue_size=10)
		#Publisher of current flapper goal for animation
		self.goal_publisher = rospy.Publisher('flapper_goal',PoseStamped,queue_size=10)
		#Subscriber to flapper pose for control
		self.pose_subscriber = rospy.Subscriber('flapper_pose',PoseStamped,self.logPose)

		#Frequency to do control loop
		self.controlHz = 20
		#Counter to use for message headers
		self.count = 1
		#If True, tries to send flapper to goal.  If False, drives straight
		self.hasGoal = False
		#Timer used for control purposes.  Lots of states in the state machine have time limits
		self.timerStart = rospy.get_time()
		#Default state for the state machine: Drive forward forever
		self.state = 'goForwardIndef'

		#Angle tolerance for rotation
		self.theta_tolerance = .1
		#Angle to set off-arm when doing turning gait
		self.limpangle = -pi/3

		#Holds goal location for control purposes
		self.goal_x = 0
		self.goal_y = 0
		self.goal_w = 0
		#Holds current location for control purposes
		self.current_x = 0
		self.current_y = 0
		self.current_w = 0
		#Location storage for driving forward state.  Lets us know when we've gone far enough
		self.storeLoc_x = 0
		self.storeLoc_y = 0
		self.goDistance = 0

		#Lets us know if we know the robot's current pose.  If not, control is pointless
		self.hasPoseData = False

		#Joint angle storage for state machine calculations
		self.currentJoints = [0,0,0,0]
		self.lastJoints = [0,0,0,0]
		self.desiredJoints = [0,0,0,0]

		#Predefined joint angle message to make publishing easier later
		self.joints = Float32MultiArray()
		mydim = MultiArrayDimension()
		mydim.label = 'joints'
		mydim.size = 4
		mydim.stride = 4
		self.joints.layout.dim.append(mydim)

		#Predefined goal pose message to make publishing easier later
		self.goalpose = PoseStamped()
		self.goalpose.header.frame_id = 'base_frame'

		#Control loop timer
		self.control_timer = rospy.Timer(rospy.Duration(1.0/self.controlHz),self.control_callback)

		#Service definition to provide new goal
		self.setgoalService = rospy.Service('set_swim_goal', SetGoal, self.setFlapperGoal)

		rospy.loginfo('Started controller server')

	#Publishes current flapper goal for the animator
	def publishGoal(self):
		
		mypose = self.goalpose
		mypose.header.stamp = rospy.Time.now()

		mypose.pose.position.x = self.goal_x
		mypose.pose.position.y = self.goal_y

		quat = tf.transformations.quaternion_from_euler(0,0,self.goal_w)
		mypose.pose.orientation.x = quat[0]
		mypose.pose.orientation.y = quat[1]
		mypose.pose.orientation.z = quat[2]
		mypose.pose.orientation.w = quat[3]

		mypose.header.seq = self.count
		self.count += 1

		#Uses one of the orientation boxes to store whether to animate the goal
		#Only get away with this because we're in 2D and all rotation will be around Z-axis
		if self.hasGoal:
			mypose.pose.orientation.x = 1
		else:
			mypose.pose.orientation.x = 0

		self.goal_publisher.publish(mypose)


	#Subscriber callback to log current flapper location/orientation
	def logPose(self,msg):
		self.current_x = msg.pose.position.x
		self.current_y = msg.pose.position.y
		quat = (msg.pose.orientation.x,
				msg.pose.orientation.y,
				msg.pose.orientation.z,
				msg.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		self.current_w = euler[2]
		self.hasPoseData = True

	#Service callback to set new flapper goal
	def setFlapperGoal(self,msg):

		try:
			#Check and see if we even want to do this goal, or if it's just to clear old goal
			if msg.do_goal == 0:
				self.goal_x = 0
				self.goal_y = 0
				self.goal_w = 0
				self.hasGoal = False
				self.timerStart = rospy.get_time()
				self.state = 'goForwardIndef'
				return SetGoalResponse(True)

			self.goal_x = msg.x_goal
			self.goal_y = msg.y_goal
			self.goal_w = msg.w_goal

			self.state = 'initiate_control'

			self.timerStart = rospy.get_time()
			self.hasGoal = True
			return SetGoalResponse(True)
		except:
			return SetGoalResponse(False)

	#Get angular error between where we're pointing and our goal location
	def getTurnAngle(self):

		dtheta = atan2(self.goal_y-self.current_y,self.goal_x-self.current_x)
		dtheta = self.current_w-dtheta

		#Wrap to (-pi,pi)
		if dtheta > pi:
			dtheta -= 2*pi
		elif dtheta < -pi:
			dtheta += 2*pi

		return dtheta

	#Meat of controller.  Drives the state machine that does control
	def control_callback(self,mytime):

		#Let the animator know where we're going
		self.publishGoal()

		#Load default joint data just in case we don't make any changes
		joints = self.joints
		joints.data = self.currentJoints

		#Shorthand of angle to drive off-arm limp wrist to during turns
		la = self.limpangle

		#Useful for debugging state machine
		rospy.loginfo(self.state)

		#State that drives the robot forward indefinitely using a forward gait
		if self.state == 'goForwardIndef':
			mytime = rospy.get_time()-self.timerStart
			mytime = (mytime%3)/3
			if mytime < .5:
				mytime = mytime*2
				joints.data = [0, -mytime*pi/2, -mytime*pi/2, 0]
			elif mytime < .75:
				mytime = (mytime-.5)*4
				joints.data = [-mytime*pi/2,(-1+mytime)*pi/2,(-1+mytime)*pi/2,-mytime*pi/2]
			else:
				mytime = (mytime-.75)*4
				joints.data = [(-1+mytime)*pi/2,0,0,(-1+mytime)*pi/2]

		#First state of goal control:  Decides if we need to turn or if we just want to go straight
		elif self.state == 'initiate_control':

			dtheta = self.getTurnAngle()

			if abs(dtheta) <= self.theta_tolerance:
				self.state = 'getFlat'
				self.timerStart = rospy.get_time()
				self.lastJoints = self.currentJoints
			else:
				self.state = 'setTurn'
				self.timerStart = rospy.get_time()
				#Shorthand works well here.  Fan of the Faces song.
				self.desiredJoints = [0,0,la,la]
				self.lastJoints = self.currentJoints

		#State that sends flapper to stretched-out pose for going straight over 1 second
		elif self.state == 'getFlat':

			mytime = rospy.get_time()-self.timerStart
			if mytime < 1:
				last_J = np.array(self.lastJoints)
				command_J = (1-mytime)*last_J
				joints.data = command_J.tolist()
			else:
				joints.data = [0,0,0,0]
				self.state = 'goForward'
				self.timerStart = rospy.get_time()
				self.storeLoc_x = self.current_x
				self.storeLoc_y = self.current_y
				self.goDistance = sqrt((self.current_x-self.goal_x)**2+(self.current_y-self.goal_y)**2)

		#State that sends flapper to turning pose over 1 second
		elif self.state == 'setTurn':

			mytime = rospy.get_time()-self.timerStart
			if mytime < 1:
				last_J = np.array(self.lastJoints)
				to_J = np.array(self.desiredJoints)
				command_J = (to_J-last_J)*mytime + last_J
				joints.data = command_J.tolist()
			else:
				joints.data = self.desiredJoints
				self.state = 'doTurn'
				self.timerStart = rospy.get_time()

		#State that turns flapper using turning gait until we're pointed in the right direction
		elif self.state == 'doTurn':

			dtheta = self.getTurnAngle()
			mytime = rospy.get_time()-self.timerStart

			if abs(dtheta) < self.theta_tolerance:
				self.state = 'getFlat'
				self.timerStart = rospy.get_time()
				self.lastJoints = self.currentJoints
			else:
				mytime = (mytime%5)/5
				if mytime < .5:
					mytime = mytime*2
					joints.data = [0,-mytime*pi/2,la,la]
				elif mytime < .75:
					mytime = (mytime-.5)*4
					joints.data = [-mytime*pi/2,(-1+mytime)*pi/2,la,la]
				else:
					mytime = (mytime-.75)*4
					joints.data = [(-1+mytime)*pi/2,0,la,la]

		#State that drives the robot forward a set distance to try to reach goal location
		elif self.state == 'goForward':

			distanceGone = sqrt((self.storeLoc_x-self.current_x)**2+
								(self.storeLoc_y-self.current_y)**2)

			if distanceGone < self.goDistance:
				mytime = rospy.get_time()-self.timerStart
				mytime = (mytime%3)/3
				if mytime < .5:
					mytime = mytime*2
					joints.data = [0, -mytime*pi/2, -mytime*pi/2, 0]
				elif mytime < .75:
					mytime = (mytime-.5)*4
					joints.data = [-mytime*pi/2,(-1+mytime)*pi/2,(-1+mytime)*pi/2,-mytime*pi/2]
				else:
					mytime = (mytime-.75)*4
					joints.data = [(-1+mytime)*pi/2,0,0,(-1+mytime)*pi/2]
			else:
				self.state = 'getFlatFinal'
				self.timerStart = rospy.get_time()
				self.lastJoints = self.currentJoints

		#State that sends robot to stretched out position when we think we're at goal location
		elif self.state == 'getFlatFinal':

			mytime = rospy.get_time()-self.timerStart
			if mytime < 1:
				last_J = np.array(self.lastJoints)
				command_J = (1-mytime)*last_J
				joints.data = command_J.tolist()
			else:
				joints.data = [0,0,0,0]
				#This isn't actually a state, so ends the control loop.  If you want cycle to repeat,
				#Change this to 'initiate_control'
				self.state = 'done'

		self.currentJoints = joints.data
		self.joint_publisher.publish(joints)




if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('controller')

	# Create instance of class that will run service
	controller = Controller()

	# Give control over to ROS.
	rospy.spin()
