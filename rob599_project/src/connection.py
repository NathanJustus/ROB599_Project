#!/usr/bin/env python

# Node to turn joint data into pose data using the connection field
# Also provides service to load connection data from matlab file

# Import the good stuff
import rospy
import sys
import tf
import rospkg
import scipy.io as sio
import numpy as np
from math import sin,cos,pi,floor,ceil
from scipy.interpolate import griddata

#Import custom and predefined message definitions
from rob599_project.srv import LoadConnection, LoadConnectionRequest, LoadConnectionResponse
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

#Class definition that will run all the functionality of the node+service
class ConnectionEvaluator:

	#Called when instance of class is initialized
	def __init__(self):

		#Tests to see if we've actually loaded the connection data.  Deprecated - Data loads automatically now
		self.loadedData = False
		#Tests to see if we have two pairs of joints so we can calculate change between them
		self.haveLastJoints = False
		#Useful to get connection files location
		self.pack = rospkg.RosPack()
		self.folderLocation = self.pack.get_path('rob599_project')
		self.fileLocation = self.folderLocation + '/connection_files/'
		rospy.loginfo(self.fileLocation)

		#Default loads 4D Flapper data from matlab file
		self.startData = LoadConnectionRequest()
		self.startData.fileName = 'Flapper_4D_PythonFriendly.mat'
		self.loadConnection(self.startData)

		#Predefine message definition for current pose
		self.pose = PoseStamped()
		self.pose.header.frame_id = 'base_frame'

		#Initialize pose values
		self.w = 0
		self.x = 3
		self.y = 3

		#Counter for message headers
		self.count = 0

		#Makes 2D flapper behave more like how a 3D flapper would behave
		#Set scale = 1 and stopBackslide = False for doing actual data analysis instead of pretty visuals
		self.stopBackslide = True
		self.scale = 3
		self.backRatio = 1./2

		#Subscriber to joint data from controller
		self.jointSubscriber = rospy.Subscriber('joint_angles', Float32MultiArray, self.propogatePose)
		#Publisher of flapper pose for controller/visualizer
		self.posePublisher = rospy.Publisher('flapper_pose',PoseStamped,queue_size=10)
		#Service to load connection data if we want to use a different file
		self.loaderService = rospy.Service('load_connection', LoadConnection, self.loadConnection)

		rospy.loginfo('Started connection server')

	#Service callback that loads connection data from specified file
	def loadConnection(self,msg):

		try:
			filename = msg.fileName
			fullfile = self.fileLocation + filename
			rospy.loginfo(fullfile)
			mymat = sio.loadmat(fullfile)
			self.jointangles = np.transpose(mymat['alphas'])
			self.connection = np.transpose(mymat['connections'])
			self.loadedData = True
			rospy.loginfo('Loaded connection data from file')
			return LoadConnectionResponse(True)
		except:
			rospy.loginfo('Something went wrong with the load process')
			return LoadConnectionResponse(False)

	#Meat of connection node - gets joint data, finds joint changes, uses to calculate spatial displacement
	#Configured as callback to joint message.
	def propogatePose(self,msg):

		joints = np.array(msg.data)

		#If we don't have old joint data to diff of of, store this data for the future and quit
		if not self.haveLastJoints:
			self.lastJoints = joints
			self.haveLastJoints = True
			return

		#If we don't have a connection loaded, quit
		if not self.loadedData:
			self.lastJoints = joints
			return

		#Calculate shape velocities
		dangles = joints - self.lastJoints
		#Calculate middle point of shape change (where we calculate connection)
		centerAngles = (joints+self.lastJoints)/2
		#Find the connection at the desired point
		A = self.getConnection(centerAngles)
		#do spacial_velocity = connection*shape_velocity
		dPose = np.matmul(A,np.transpose(dangles))
		#Scale displacement up to get more satisfying movement (cheating, disable in __init__ if not just doing pretty visuals)
		dPose = self.scale*dPose
		#Flapper body frame in sysplotter is 90 degrees off from body frame I'm using, so rotate it using transformation matrix
		toFlapperFrame = np.array([[0,-1,0],[1,0,0],[0,0,1]])
		dPose = np.transpose(np.matmul(dPose,toFlapperFrame))
		#Stop flapper from sliding backwards in water so much (cheating, disable in __init__ if not just doing pretty visuals)
		if self.stopBackslide and dPose[0] < 0:
			dPose[0] = dPose[0]*self.backRatio
		mid_w = self.w + dPose[2]/2
		#Send displacement from flapper body frame to world frame
		toWorldFrame = np.array([[cos(mid_w),-sin(mid_w),0],
								[sin(mid_w),cos(mid_w),0],
								[0,0,1]])
		dPose = np.transpose(np.matmul(toWorldFrame,np.transpose(dPose)))

		self.x += dPose[0]
		self.y += dPose[1]
		self.w += dPose[2]

		#If we left the screen, put us back in the middle.
		if abs(self.x) > 6 or abs(self.y) > 6 or self.x < 0 or self.y < 0:
			self.x = 3
			self.y = 3
			self.w = 0

		#Send out message with new pose
		mypose = self.pose
		mypose.header.stamp = rospy.Time.now()
		mypose.header.seq = self.count
		self.count += 1

		quat = tf.transformations.quaternion_from_euler(0,0,self.w)
		mypose.pose.orientation.x = quat[0]
		mypose.pose.orientation.y = quat[1]
		mypose.pose.orientation.z = quat[2]
		mypose.pose.orientation.w = quat[3]

		mypose.pose.position.x = self.x
		mypose.pose.position.y = self.y

		self.lastJoints = joints

		self.posePublisher.publish(mypose)

	#Calculates connection at a given shape using linear interpolation of sysplotter data
	def getConnection(self,angles):
		#Cut interpolation points down from 4096 values to scan through, to 16
		relevant_connection, relevant_angles = self.prepInterp(angles) 
		#Interpolate connection at desired shape from sysplotter data
		value = griddata(relevant_angles,relevant_connection,angles)
		#Format connection from row-vector to 3x4 matrix
		thisConnection = np.reshape(value,(4,3))
		thisConnection = np.transpose(thisConnection)
		return thisConnection

	#Cuts interpolation points from 4096 to 16
	#Speeds up interpolation to be super fast.  Scanning all 4096 points takes ~2 seconds.  Way too slow
	#We get away with this because we know the structure of the connection field data from sysplotter and can choose the best points
	def prepInterp(self,angles):

		index_list = []

		a1 = self.convertToIndex(angles[0])
		a2 = self.convertToIndex(angles[1])
		a3 = self.convertToIndex(angles[2])
		a4 = self.convertToIndex(angles[3])

		for a1_val in a1:
			for a2_val in a2:
				for a3_val in a3:
					for a4_val in a4:
						this_index = a1_val + 8*a2_val + 8*8*a3_val + 8*8*8*a4_val
						this_index = int(this_index)
						index_list.append(this_index)

		filtered_connection = np.array([])
		filtered_angles = np.array([])
		for index in index_list:
			filtered_connection = np.append(filtered_connection,self.connection[index])
			filtered_angles = np.append(filtered_angles,self.jointangles[index])
		filtered_connections = np.reshape(filtered_connection,(16,12))
		filtered_angles = np.reshape(filtered_angles,(16,4))

		return filtered_connections,filtered_angles

	#Takes a shape value and finds the index of upper and lower points in sysplotter connection data
	def convertToIndex(self,angle):
		low = floor((angle+2)*7/4)
		high = ceil((angle+2)*7/4)
		return [low,high]





if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('connection_evaluator')

	# Create instance of class that will run both services
	connection_eval = ConnectionEvaluator()

	# Give control over to ROS.
	rospy.spin()
