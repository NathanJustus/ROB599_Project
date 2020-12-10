#!/usr/bin/env python

# Node to publish fake joint and pose data for validation purposes

# Import the good stuff
import rospy
import sys
import tf
from math import pi

#Import predefined ROS message definitions
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped

#Class definition that will run all the functionality of the node
class DataFaker:

	#Called when instance of class is initialized
	def __init__(self):

		#Declare publisher of joint data for testing
		self.joint_publisher = rospy.Publisher('joint_angles',Float32MultiArray,queue_size=10)
		#Declare publisher of pose data for testing
		self.pose_publisher = rospy.Publisher('flapper_pose',PoseStamped,queue_size=10)

		#Booleans to decide which data to spoof
		self.doJoints = True;
		self.doPose = False;
		#Frequencies at which to spoof data
		self.jointHz = 20;
		self.poseHz = 20;

		#Counter for labelling message headers
		self.count = 0

		#Declare timer callbacks to publish spoofed data at designated frequencies
		self.joint_timer = rospy.Timer(rospy.Duration(1.0/self.jointHz),self.joint_callback)
		self.pose_timer = rospy.Timer(rospy.Duration(1.0/self.poseHz),self.pose_callback)

		#Predefine message that we'll send that holds joint values
		self.joints = Float32MultiArray()
		mydim = MultiArrayDimension()
		mydim.label = 'joints'
		mydim.size = 4
		mydim.stride = 4
		self.joints.layout.dim.append(mydim)

		#Predefine message to hold swimmer poses
		self.pose = PoseStamped()
		self.pose.header.frame_id = 'base_frame'

		#Announce that the node started
		rospy.loginfo('Started data faking node')

	#Update joint data test function  Does a basic forward-swimming gait
	def joint_callback(self,mytime):

		if not self.doJoints:
			return

		joints = self.joints
		
		mytime = (rospy.get_time()%3)/3
		if mytime < .5:
			mytime = mytime*2
			joints.data = [0, -mytime*pi/2, -mytime*pi/2, 0]
		elif mytime < .75:
			mytime = (mytime-.5)*4
			joints.data = [-mytime*pi/2,(-1+mytime)*pi/2,(-1+mytime)*pi/2,-mytime*pi/2]
		else:
			mytime = (mytime-.75)*4
			joints.data = [(-1+mytime)*pi/2,0,0,(-1+mytime)*pi/2]

		self.joint_publisher.publish(joints)

	#Update pose data test function.  Slides swimmer up and right while rotating
	def pose_callback(self,mytime):

		if not self.doPose:
			return

		rotate = True
		translate = True

		mypose = self.pose
		mypose.header.stamp = rospy.Time.now()
		mypose.header.seq = self.count
		self.count += 1

		mytime = (rospy.get_time()%3)/3

		if rotate:
			quat = tf.transformations.quaternion_from_euler(0,0,2*pi*mytime)
			mypose.pose.orientation.x = quat[0]
			mypose.pose.orientation.y = quat[1]
			mypose.pose.orientation.z = quat[2]
			mypose.pose.orientation.w = quat[3]

		if translate:
			mypose.pose.position.x = mytime*3
			mypose.pose.position.y = mytime*3

		self.pose_publisher.publish(mypose)

if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('data_faker')

	# Create instance of class
	faker = DataFaker()

	# Give control over to ROS.
	rospy.spin()
