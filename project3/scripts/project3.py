#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.

#@author: Santosh Balajee Banisetty


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

import tf
import math

from baxter_interface import Gripper

def move_group_python_interface_tutorial():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    listener = tf.TransformListener()


    try:
        listener.waitForTransform('/world', '/pick', rospy.Time(0),rospy.Duration(10000.0))
        (pick_trans, pick_rot) = listener.lookupTransform('/world', '/pick', rospy.Time(0))
        print (pick_trans)
        print (pick_rot)
        listener.waitForTransform('/world', '/place', rospy.Time(0),rospy.Duration(10000.0))
        (place_trans, place_rot) = listener.lookupTransform('/world', '/place', rospy.Time(0))
        print (place_trans)
        print (place_rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Exception in tf"
    

    left_gripper = Gripper('left')

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    group = moveit_commander.MoveGroupCommander("left_arm")


    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(5)
    print "============ Starting Project 3 "

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()

    ## We can also print the name of the end-effector link for this group
    print "============ End-effector link: %s" % group.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    print "============ Generating plan 0"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = pick_rot[0]
    pose_target.orientation.y = pick_rot[1]
    pose_target.orientation.z = pick_rot[2]
    pose_target.orientation.w = pick_rot[3]
    pose_target.position.x = pick_trans[0]
    pose_target.position.y = pick_trans[1]
    pose_target.position.z = pick_trans[2] + 0.1
    group.set_pose_target(pose_target)

    plan0 = group.plan()

    print "============ Waiting while RVIZ displays plan0..."
    rospy.sleep(5)

    group.go(wait=True)

    rospy.sleep(1)

    left_gripper.open(block=False, timeout=5.0)

    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = pick_rot[0]
    pose_target.orientation.y = pick_rot[1]
    pose_target.orientation.z = pick_rot[2]
    pose_target.orientation.w = pick_rot[3]
    pose_target.position.x = pick_trans[0]
    pose_target.position.y = pick_trans[1]
    pose_target.position.z = pick_trans[2]
    group.set_pose_target(pose_target)

    ## Now, we call the planner to compute the plan
    ## and visualize it if successful
    ## Note that we are just planning, not asking move_group 
    ## to actually move the robot
    plan1 = group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)

   
    group.go(wait=True)

    rospy.sleep(1)

    left_gripper.close(block=False, timeout=5.0)

    pose_target_z = geometry_msgs.msg.Pose()
    pose_target_z.orientation.x = pick_rot[0]
    pose_target_z.orientation.y = pick_rot[1]
    pose_target_z.orientation.z = pick_rot[2]
    pose_target_z.orientation.w = pick_rot[3]
    pose_target_z.position.x = pick_trans[0]
    pose_target_z.position.y = pick_trans[1]
    pose_target_z.position.z = pick_trans[2] + 0.1
    group.set_pose_target(pose_target_z)

    plan2 = group.plan()

    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)

    group.go(wait=True)

    pose_target1_z = geometry_msgs.msg.Pose()
    pose_target1_z.orientation.x = place_rot[0]
    pose_target1_z.orientation.y = place_rot[1]
    pose_target1_z.orientation.z = place_rot[2]
    pose_target1_z.orientation.w = place_rot[3]
    pose_target1_z.position.x = place_trans[0]
    pose_target1_z.position.y = place_trans[1]
    pose_target1_z.position.z = place_trans[2] + 0.1
    group.set_pose_target(pose_target1_z)

    plan3 = group.plan()

    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)

    group.go(wait=True)

    pose_target1 = geometry_msgs.msg.Pose()
    pose_target1.orientation.x = place_rot[0]
    pose_target1.orientation.y = place_rot[1]
    pose_target1.orientation.z = place_rot[2]
    pose_target1.orientation.w = place_rot[3]
    pose_target1.position.x = place_trans[0]
    pose_target1.position.y = place_trans[1]
    pose_target1.position.z = place_trans[2]
    group.set_pose_target(pose_target1)

    plan4 = group.plan()

    print "============ Waiting while RVIZ displays plan4..."
    rospy.sleep(5)

    group.go(wait=True)
    rospy.sleep(1)
    left_gripper.open(block=False, timeout=5.0)

    group.clear_pose_targets()

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

    print "============ STOPPING"


if __name__=='__main__':

    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
