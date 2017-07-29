#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from traj_msgs.msg import TrajectoryMultiCommand
from traj_msgs.msg import TrajectoryCommand

trajectoryMultiCmd = TrajectoryMultiCommand()

def msg_pub():
    pub1 = rospy.Publisher('/part1_message_publisher', TrajectoryCommand, queue_size=10)
    pub2 = rospy.Publisher('/part2_message_publisher', TrajectoryMultiCommand, queue_size=10)
    rospy.init_node('message_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)

        trajectoryCmd = TrajectoryCommand()
        trajectoryCmd.time = rospy.Time.now()
        trajectoryCmd.t_k.data = rospy.Duration(2, 0)
        trajectoryCmd.t_k_prime.data = rospy.Duration(2, 0)
        trajectoryCmd.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
        trajectoryCmd.q_final = [1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0]
        trajectoryCmd.qdot_final = [0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0]

        trajectoryMultiCmd.points.append(trajectoryCmd)

        pub1.publish(trajectoryCmd)
        pub2.publish(trajectoryMultiCmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        msg_pub()
    except rospy.ROSInterruptException:
        pass