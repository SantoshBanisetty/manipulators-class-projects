#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')

import rospy
import tf
from mv_msgs.msg import move_to_from

x = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
y = ['1', '2', '3', '4', '5', '6', '7', '8']

pickString = "a1"
placeString = "a1"

def callback(data):
    global pickString, placeString
    rospy.loginfo(rospy.get_caller_id() + "I heard a pick and place location")
    pickString = data.move_from
    placeString = data.move_to

if __name__ == '__main__':
    rospy.init_node('tf_fix_broadcaster')
    rospy.Subscriber("pick_and_place", move_to_from, callback)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print (pickString, placeString)
        picklst = list(pickString)
        pickX = x.index(picklst[0]) + 1
        pickY = y.index(picklst[1]) + 1
        placelst = list(placeString)
        placeX = x.index(placelst[0]) + 1
        placeY = y.index(placelst[1]) + 1
        
        br.sendTransform((pickX * 0.0625 - 0.03125, pickY * 0.0625 - 0.03125, 0.0),
                         (1.0, 0.0, 0.0, 0.0),
                         rospy.Time.now(),
                         "pick",
                         "checkerboard_corner")
        # br.sendTransform((0.5, 0.25, 0.0),
        #                  (1.0, 0.0, 0.0, 0.0),
        #                  rospy.Time.now(),
        #                  "pick",
        #                  "checkerboard_corner")

        br.sendTransform((placeX * 0.0625 - 0.03125, placeY * 0.0625 - 0.03125, 0.0),
                         (1.0, 0.0, 0.0, 0.0),
                         rospy.Time.now(),
                         "place",
                         "checkerboard_corner")
        rate.sleep()