#!/usr/bin/env python

import pudb
import sys, time
import numpy as np
import random
import copy
import baxter_interface
import math

import rospy
from math import sin, cos, pi
from baxter_core_msgs.msg import JointCommand
from geometry_msgs.msg import Point
from traj_msgs.msg import TrajectoryCommand
from traj_msgs.msg import TrajectoryMultiCommand
from traj_msgs.msg import Jacobian
from sensor_msgs.msg import JointState

from collections import defaultdict

z_lst = []
p_lst = []


THETA = 0
D = 1
A = 2
ALPHA = 3

flag = False


pretty = "=========================="
POS = 0
VEL = 1

velocity_dict = {'right_s0': 0.0, 
                'right_s1': 0.0, 
                'right_e0': 0.0, 
                'right_e1': 0.0, 
                'right_w0': 0.0, 
                'right_w1': 0.0, 
                'right_w2': 0.0}

initial_states = defaultdict(list)


#part 1 
def part1messagecallback(msg):
    rospy.loginfo("Baxter part 1")
    for name in msg.names:
        index = msg.names.index(name)
        qf = msg.q_final[index]
        qdotf = msg.q_final[index]
        temp = msg.t_k.data
        tf = temp.secs + temp.nsecs * 10**9
        #ax=b
        a = np.array([[1, 0, 0, 0],
            [0, 1, 0, 0],
            [1, tf, tf**2, tf**3],
            [0, 1, 2*tf, 3*tf**2]]) 

        b = np.array([initial_states[name][POS], initial_states[name][VEL], qf, qdotf])

        x = np.linalg.solve(a, b)
        #rospy.loginfo(x)
        velocity_dict[name] = (3*x[3]*tf**2 + 2*x[2]*tf + x[1])
    #rospy.loginfo(velocity_dict)


#part 2 b option wasn't finished as of submission
def part2messagecallback(msg):
    rospy.loginfo("Baxter part 2")
    #velocity = []
    '''qf = msg.q_final[1]
    qdotf = msg.qdot_final[1]
    temp = msg.t_k.data
    tf = temp.secs + temp.nsecs * 10**9'''
    for point in msg.points:
        #rospy.loginfo(point)
        for name in point.names:
            index = point.names.index(name)
            qf = point.q_final[index]
            qdotf = point.qdot_final[index]
            temp = point.t_k.data
            tf = temp.secs + temp.nsecs * 10**9
            #ax=b
            a = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [1, tf, tf**2, tf**3],
                [0, 1, 2*tf, 3*tf**2]]) 

            b = np.array([initial_states[name][POS], initial_states[name][VEL], qf, qdotf])

            x = np.linalg.solve(a, b)
            #rospy.loginfo(x)
            velocity_dict[name] = (3*x[3]*tf**2 + 2*x[2]*tf + x[1])
        #rospy.loginfo(velocity_dict)
    
#Fills the dictionary to collect state values
def filldictcallback(msg):
    global flag 
    flag = True
    for joint in msg.name:
        #rospy.loginfo(joint)
        idx = msg.name.index(joint)
        initial_states[joint] = [msg.position[idx], msg.velocity[idx]]
    #rospy.loginfo(initial_states)
    #rospy.loginfo(pretty)

    rospy.loginfo("dictionary fill")


# jacobian class
class jacobian:
    """docstring for jacobian"""
    def __init__(self):
    	pass

    def getZi_1(self, i, j):
        temp = self.getRotationMatrix(i, j)[:, 2]
        return temp

    def getPe(self, i, j):
        #return self.getTranslation(i, j)
        return self.getT(i, j).dot(np.array([0, 0, 0, 1]).transpose())

    def getPi_1(self, i, j):
        #return self.getTranslation(i, j)
        return self.getT(i, j).dot(np.array([0, 0, 0, 1]).transpose())
        
    def getTranslation(self, i,j):
        return self.getT(i,j)[:-1,-1]

    def getRotationMatrix(self, i,j):

        temp = self.getT(i, j)
        print(temp)
        print(type(temp[:3, :3]))
        return temp[:3, :3]
        #print( self.getT(i,j)[:3, :3])

    def constructA(self, jnt):
        rospy.loginfo("Constructing A")
        theta = jnt[THETA]
        d = jnt[D]
        a = jnt[A]
        alpha = jnt[ALPHA]
        #rospy.loginfo(jnt)
        return np.array([(math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)), 
                (math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)), 
                (0, math.sin(alpha), math.cos(alpha), d), 
                (0, 0, 0, 1)])

    def getT(self, i, j):
    # TODO: complete this function to get a transform from the ith frame to the jth frame and return it (hont: use constructA above)
        rospy.loginfo("flag : %s", flag)
        if flag == True:
            dh = {0 : [initial_states['right_s0'][POS], 0.27035, 0.069,-pi/2], 
            1 : [initial_states['right_s1'][POS] + pi/2, 0, 0, pi/2],       
            2 : [initial_states['right_e0'][POS], 0.36435, 0.0690, -pi/2], 
            3 : [initial_states['right_e1'][POS], 0, 0, pi/2],           
            4 : [initial_states['right_w0'][POS], 0.37429, 0.010, -pi/2],  
            5 : [initial_states['right_w1'][POS], 0, 0, pi/2],          
            6 : [initial_states['right_w2'][POS], 0.229525, 0, 0]}

            trnsfrm = np.identity(4)
            #test = trnsfrm.dot(self.constructA(dh[2]))  
            #print(test[:3,:3])
            for x in range (i, j):
        	    #print(x)
        	    #trnsfrm = np.matmul(trnsfrm, self.constructA(x))
                #print(pretty)
                #print("Type of A", type(self.constructA(dh[x])))
                #print(self.constructA(dh[x])[:3, :3])
                trnsfrm = trnsfrm.dot(self.constructA(dh[x]))
                #print(trnsfrm[:3,:3])
            return trnsfrm





def vel_test():
    jointCmdPub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 100)
    jacobianPub = rospy.Publisher("/jacobian_publisher", Jacobian, queue_size = 100)
    rospy.init_node('vel_test')
    rospy.Subscriber("/part1_message_publisher", TrajectoryCommand, part1messagecallback)
    #rospy.Subscriber("/part2_message_publisher", TrajectoryMultiCommand, part2messagecallback)
    rospy.Subscriber("/robot/joint_states", JointState, filldictcallback)
    rate = rospy.Rate(1000)
    #i = 0
    while not rospy.is_shutdown():
        #rospy.loginfo(":-)")
        jabpub = Jacobian()
        jointCmd = JointCommand()
        jointCmd.mode = JointCommand.TORQUE_MODE
        jointCmd.names.append('right_s0')
        jointCmd.names.append('right_s1')
        jointCmd.names.append('right_e0')
        jointCmd.names.append('right_e1')
        jointCmd.names.append('right_w0')
        jointCmd.names.append('right_w1')
        jointCmd.names.append('right_w2')
        for name in jointCmd.names:
            jointCmd.command.append( velocity_dict[name] )
            #rospy.loginfo(name)
            '''if i < 4000:
                vel = i / 1000.0
                jointCmd.command.append( vel )
            else:
                jointCmd.command.append(0.0)
        i = i+1'''
        jointCmdPub.publish(jointCmd)
        

        #print("type of T",type(jab.getT(0,2)))
        if flag == True:
            jab = jacobian()
            for idx in xrange(0,8):
            #print (idx)
                rospy.loginfo(jab)
                z_lst.append(jab.getZi_1(0, idx))
                p_lst.append(jab.getPi_1(0, idx))
                
            pe = jab.getPe(0, 7)
            print (pretty)
            print ("pe", pe)
            for x in xrange(0, 8):
                jabpub.names = []
                print ("z", z_lst[x])
                print (pretty)
                print ("p", p_lst[x])
                #dimension error
                #jabpub.JP.append(z_lst[x].transpose().dot((pe - p_lst[x])))
            	jabpub.JO.append(z_lst[x])
                jacobianPub.publish(jabpub)
        #print (pretty)
        #print(z_lst)
        #print (pretty)
        #print(p_lst)
        global z_lst, p_lst
        z_lst = []
        p_lst = []
        rate.sleep()


if __name__ == '__main__':
    try:
        vel_test()
    except rospy.ROSInterruptException:
        pass