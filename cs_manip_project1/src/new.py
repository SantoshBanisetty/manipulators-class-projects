#!/usr/bin/env python

#Project 1
#Submission by the following students
#Santosh Balajee Banisetty
#Tanvir Ahmed Siddiqui
#and Meera Sebastian 

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import rospy
import math

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

pointsArray = MarkerArray()
links = Marker()

p = []

lkey = 0

position = [0,0,0,0,0,0,0]

path = "/home/santosh/catkin_ws/src/cs_manip_project1/"

class MyRobot:

	def __init__(self, filename):
		f = open(path + filename)
		botSpecs = eval(f.read())
		f.close()
		self.parseDHFile(botSpecs)

	def parseDHFile(self, DHparams):
		# list of 4-tuples (a, alpha, d, theta)
		if type(DHparams) == dict:
			self.setDHFromDict(DHparams)
		elif type(DHparams) == list:
			self.dhParams = DHparams
		else:
			throw("Invalid Format")

		#print self.dhParams

		self.numLinks = len(self.dhParams)
		self.q = np.zeros((self.numLinks, 1))

	def setDHFromDict(self, DHdict):
		self.dhParams = []
		for joint in DHdict["Joints"]:
			self.dhParams.append(joint["DH_parameters"])

	def getTranslation(self, i,j):
		return self.getT(i,j)[:-1,-1]

	def getRotationMatrix(self, i,j):
		return self.getT(i,j)[:3,:3]


	# constructA: constructs the transform matrix for the ith joint using the D-H parameters
	# i: the joint to construct the homogeneous transform for
	# returns the transform from the ith to the i+1th
	def constructA(self, i):
		a, alpha, d ,theta = self.dhParams[i]
		# TODO: complete this function to get a homogeneous transform from frame i to i+1 using the D-H params and return it
		theta = position[i]
		#print (theta)
		return np.array([(math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)), 
			(math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)), 
			(0, math.sin(alpha), math.cos(alpha), d), 
			(0, 0, 0, 1)])



	# getT: computes a homogeneous transform from the ith frame to the jth frame by iteratively multiplying each homogeneous transform from i to j
	# i is the index of the starting coordinate frame
	# j is the index of the ending coordinate frame
	# returns: a homogeneous transform matrix, T

	def getT(self, i, j):
		# TODO: complete this function to get a transform from the ith frame to the jth frame and return it (hont: use constructA above)
		trnsfrm = np.identity(4)
		for x in range (i, j):
			#print(x)
			#trnsfrm = np.matmul(trnsfrm, self.constructA(x))
			trnsfrm = trnsfrm.dot(self.constructA(x))
		return trnsfrm


def jointStateCallback(data):
	rospy.loginfo("I was called")
	temp = list(data.position)
	for idx, x in enumerate(temp):
		position [idx] = x





#function for marker joints
def joints():
	global jkey
	for index in xrange(1, 8):
		
		#rospy.loginfo("I am doing just O.K.")
		points = Marker()
		points.header.frame_id = "/base_link"
		points.header.stamp = rospy.Time.now()
		#rospy.loginfo(type(rospy.get_time))
		points.ns = "j_markers"+str(index)
		#print("namespace", points.ns)
		points.action = Marker.ADD
		#points.pose.orientation.w = 1.0
		points.id = 0
		points.type = Marker.CUBE
		points.scale.x = 0.02
		points.scale.y = 0.02
		points.scale.z = 0.02
		points.color.b = 1.0
		points.color.a = 1.0

		points.pose.position.x = p[index][0]
		points.pose.position.y = p[index][1]
		points.pose.position.z = p[index][2]

		points.pose.orientation.w = p[index][3]
		points.pose.orientation.x = p[index][4]
		points.pose.orientation.y = p[index][5]
		points.pose.orientation.z = p[index][6]
		#print ("jkey", jkey)
		#rospy.loginfo()
		pointsArray.markers.append(points)

	#rospy.loginfo(pointsArray)
	point_publisher.publish(pointsArray)
'''
	jkey += 1
	#rospy.loginfo(jkey)
	if jkey == 8:
		#rospy.loginfo("got you")
		jkey = 1
		point_publisher.publish(pointsArray)
		#print(pointsArray)'''

#Function for marker links between joints
def linksMark(links):
	global lkey
	links.points = []
		
	#rospy.loginfo("I am doing just O.K.")
		
	links.header.frame_id = "/base_link"
	links.header.stamp = rospy.Time.now()
	
	links.ns = "l_markers"
	links.action = Marker.ADD
	#links.pose.orientation.w = 1.0
	links.id = 1
	links.type = Marker.LINE_STRIP
	links.scale.x = 0.02
	links.scale.y = 0.02
	links.scale.z = 0.02
	links.color.g = 1.0
	links.color.a = 1.0

	for lkey in range(0,8):
		pts = Point()
		pts.x = p[lkey][0]
		pts.y = p[lkey][1]
		pts.z = p[lkey][2]

		links.points.append(pts)

		#rospy.loginfo(lkey)

	link_publisher.publish(links)
		

if __name__ == '__main__':
	try:
		point_publisher = rospy.Publisher('visualization_joint_marker_array', MarkerArray, queue_size=10)
		link_publisher = rospy.Publisher('visualization_link_marker', Marker, queue_size=10)
		rospy.Subscriber("/joint_state", JointState , jointStateCallback)
		rospy.init_node('markers', anonymous = True)
		rate = rospy.Rate(10)
		rotMat = np.zeros((3,3))
		
		while not rospy.is_shutdown():
			#print(position)
			#print(len(position))
			bot = MyRobot("robot_test.json")
			for idx in xrange(0,8):
				#print (0, idx)
				lst = bot.getTranslation(0, idx)
				#print (lst)
				rotMat = bot.getRotationMatrix(0, idx)
				#print (rotMat)
				qw = (math.sqrt(1+rotMat[0,0]+rotMat[1,1]+rotMat[2,2]))/2.0
				qx = np.sign(rotMat[2,1] - rotMat[1,2]) * (math.sqrt(1+rotMat[0,0]-rotMat[1,1]-rotMat[2,2])) * 0.5
				qy = np.sign(rotMat[0,2] - rotMat[2,0]) * (math.sqrt(1+rotMat[0,0]-rotMat[1,1]-rotMat[2,2])) * 0.5
				qz = np.sign(rotMat[1,0] - rotMat[0,1]) * (math.sqrt(1+rotMat[0,0]-rotMat[1,1]-rotMat[2,2])) * 0.5
				lst = np.append(lst, [qw, qx, qy, qz])
				#print(lst)
				p.append(lst)
				#print (bot.getTranslation(0, idx))
			
			joints()
			
			linksMark(links)

			#print(p)
			p = []
	#position = []
			rate.sleep()
	except rospy.ROSInterruptException:
		pass