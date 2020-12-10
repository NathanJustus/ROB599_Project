#!/usr/bin/env python

# Node to visualize swimming flapper data

# Import the good stuff
import rospy
import sys
from math import pi, sin, cos
import tf
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Import message definitions
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

#Class definition that will run all the functionality of the node
class Flapper_Visualizer:

	#Called when instance of class is initialized
	def __init__(self):

		#Set default pose data
		self.pose_x = 3
		self.pose_y = 3
		self.pose_w = 0

		#Scale of flapper.  Increase to make flapper thicc
		self.scale = 1
		#Ratio of ellipse paddle major/minor axes
		self.aspect_ratio = 1.0/5
		#Ratio of length/width of flapper 'arm' rectangles
		self.armshrink = 1.0/15

		#Set default joint data
		self.joints = [0,0,0,0]

		#Format animation window
		self.fig = plt.figure(figsize=(8,8))
		self.ax = self.fig.add_subplot(111)
		self.ax.axis('equal')
		self.fig.show()

		#Set default goal location
		self.goal_x = 0
		self.goal_y = 0
		self.goal_w = 0
		#Boolean describing whether or not to animate goal
		self.hasGoal = False

		#Subscriber of goal pose to animate
		self.goalSubscriber = rospy.Subscriber('flapper_goal',PoseStamped,self.logGoal)
		#Subscriber of joint data for animation
		self.joint_subscriber = rospy.Subscriber('joint_angles', Float32MultiArray, self.logangles)
		#Subscriber of flapper current pose data for animation
		self.pose_subscriber = rospy.Subscriber('flapper_pose', PoseStamped, self.logpose)

		#Announce that the node started
		rospy.loginfo('Started visualizer node')

	#Goal subscriber callback.  Stores goal for animation
	def logGoal(self,msg):

		if msg.pose.orientation.x == 0:
			self.hasGoal = False
			return

		self.hasGoal = True
		msg.pose.orientation.x = 0
		self.goal_x = msg.pose.position.x
		self.goal_y = msg.pose.position.y
		quat = (msg.pose.orientation.x,
				msg.pose.orientation.y,
				msg.pose.orientation.z,
				msg.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		self.goal_w = euler[2]

	#Joint subscriber callback.  Stores current joint angles
	def logangles(self,msg):
		self.joints = msg.data

	#Pose subscriber callback.  Stores current flapper spatial position/orientation
	def logpose(self,msg):
		self.pose_x = msg.pose.position.x
		self.pose_y = msg.pose.position.y
		quat = (msg.pose.orientation.x,
				msg.pose.orientation.y,
				msg.pose.orientation.z,
				msg.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		w = euler[2]
		self.pose_w = w

	#Meat of animation.  Refreshes animation window to match current flapper data
	def animate_flapper(self):

		plt.cla()
		self.ax.set_aspect('equal','box')
		self.ax.set(xlim=(0,6),ylim=(0,6))
		self.ax.fill([-1,7,7,-1],[-1,-1,7,7],'tab:blue')
		self.fig.tight_layout()

		self.plotFlapperPoints(self.pose_x,self.pose_y,self.pose_w,self.joints)
		if self.hasGoal:
			self.plotFlapperPoints(self.goal_x,self.goal_y,self.goal_w,[0,0,0,0],transparent=True)

		self.fig.canvas.draw()

	#Uses transformation matrices to get animation points from stored joint/pose/goal data
	def plotFlapperPoints(self,x,y,w,joints,transparent=False):

		r = self.scale*.1
		R = np.array([[cos(w),-sin(w)],[sin(w),cos(w)]])

		#Blue sphere body
		theta = np.linspace(pi/3,2*pi-pi/3,100)
		sphere_x = r*np.cos(theta)
		sphere_y = r*np.sin(theta)
		sphere_points = np.array([sphere_x,sphere_y])
		sphere_points = np.matmul(R,sphere_points)
		sphere_points[0,:] += x
		sphere_points[1,:] += y
		#Red sphere 'face'
		theta = np.linspace(-pi/3,pi/3,25)
		face_x = r*np.cos(theta)
		face_y = r*np.sin(theta)
		face_points = np.array([face_x,face_y])
		face_points = np.matmul(R,face_points)
		face_points[0,:] += x
		face_points[1,:] += y

		#Arms
		arm_x = np.array([r,-r,-r,r])
		dy = self.armshrink*r
		arm_y = np.array([dy,dy,-dy,-dy])
		arm_points = np.array([arm_x,arm_y,np.ones(len(arm_x))])
		#Paddles
		theta = np.linspace(0,2*pi,100)
		paddle_x = r*np.cos(theta)
		paddle_y = r*self.aspect_ratio*np.sin(theta)
		paddle_points = np.array([paddle_x,paddle_y,np.ones(len(paddle_x))])
		
		#Left arm
		leftarm_points = arm_points
		t1 = self.transform(0,x,y)
		t2 = self.transform(w+pi/2,r*cos(w+pi/2),r*sin(w+pi/2))
		t3 = self.transform(-joints[1],0,0)
		t4 = self.transform(0,r,0)
		t_leftarm = np.matmul(np.matmul(t1,t2),np.matmul(t3,t4))
		leftarm_points = np.matmul(t_leftarm,leftarm_points)
		#Left Paddle
		leftpaddle_points = paddle_points
		t5 = self.transform(0,r,0)
		t6 = self.transform(-joints[0],0,0)
		t7 = self.transform(0,r,0)
		t_leftpaddle = np.matmul(np.matmul(t_leftarm,t5),np.matmul(t6,t7))
		leftpaddle_points = np.matmul(t_leftpaddle,leftpaddle_points)

		#Right arm
		rightarm_points = arm_points
		t1 = self.transform(0,x,y)
		t2 = self.transform(w-pi/2,r*cos(w-pi/2),r*sin(w-pi/2))
		t3 = self.transform(joints[2],0,0)
		t4 = self.transform(0,r,0)
		t_rightarm = np.matmul(np.matmul(t1,t2),np.matmul(t3,t4))
		rightarm_points = np.matmul(t_rightarm,rightarm_points)
		#Right paddle
		rightpaddle_points = paddle_points
		t5 = self.transform(0,r,0)
		t6 = self.transform(joints[3],0,0)
		t7 = self.transform(0,r,0)
		t_rightpaddle = np.matmul(np.matmul(t_rightarm,t5),np.matmul(t6,t7))
		rightpaddle_points = np.matmul(t_rightpaddle,rightpaddle_points)

		bcolor = 'forestgreen'
		rodcolor = 'dimgray'
		lw = .3
		if transparent:
			al = .5
		else:
			al = 1

		self.ax.fill(sphere_points[0,:],sphere_points[1,:],bcolor,alpha=al)
		self.ax.fill(face_points[0,:],face_points[1,:],'tab:red',alpha=al)
		self.ax.fill(leftarm_points[0,:],leftarm_points[1,:],rodcolor,alpha=al)
		self.ax.fill(rightarm_points[0,:],rightarm_points[1,:],rodcolor,alpha=al)
		self.ax.fill(leftpaddle_points[0,:],leftpaddle_points[1,:],bcolor,alpha=al)
		self.ax.fill(rightpaddle_points[0,:],rightpaddle_points[1,:],bcolor,alpha=al)
		self.ax.plot(sphere_points[0,:],sphere_points[1,:],'k',linewidth=lw)
		self.ax.plot(face_points[0,:],face_points[1,:],'k',linewidth=lw)
		self.ax.plot(leftarm_points[0,:],leftarm_points[1,:],'k',linewidth=lw)
		self.ax.plot(rightarm_points[0,:],rightarm_points[1,:],'k',linewidth=lw)
		self.ax.plot(leftpaddle_points[0,:],leftpaddle_points[1,:],'k',linewidth=lw)
		self.ax.plot(rightpaddle_points[0,:],rightpaddle_points[1,:],'k',linewidth=lw)

	#Generates numpy SE2 transformation matrix from rotation angle, x displacement, and y displacement
	def transform(self,w,dx,dy):
		row1 = [cos(w),-sin(w),dx]
		row2 = [sin(w),cos(w),dy]
		row3 = [0,0,1]
		return np.array([row1,row2,row3])

if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('flapper_visualizer')

	# Create instance of class
	flap_viz = Flapper_Visualizer()

	animationHz = 15
	rate = rospy.Rate(animationHz)
	#Weirdly, this has to be in the main loop and can't be set using a ROS timer callback
	#Has something to do with animators not liking multithreading
	while not rospy.is_shutdown():
		flap_viz.animate_flapper()