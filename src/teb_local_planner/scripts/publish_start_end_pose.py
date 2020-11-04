#!/usr/bin/env python

import rospy
from teb_local_planner.msg import PoseSeqMsg
from geometry_msgs.msg import Pose


def publish_start_end_pose_msg():
    pub = rospy.Publisher('/test_optim_node/start_end_pose',
                          PoseSeqMsg, queue_size=1)
    rospy.init_node("start_end_pose")

    start_end_pose = PoseSeqMsg()
    start_end_pose.header.stamp = rospy.Time.now()
    start_end_pose.header.frame_id = "odom/map"  # CHANGE HERE: odom/map

    start_end_pose.pose_seq.append(Pose())
    start_end_pose.pose_seq[-1].position.x = -6
    start_end_pose.pose_seq[-1].position.y = 2.5
    # Here we use geometry_msgs/Quaternion z field to store heading
    start_end_pose.pose_seq[-1].orientation.z = 0.0

    start_end_pose.pose_seq.append(Pose())
    start_end_pose.pose_seq[-1].position.x = 2.0
    start_end_pose.pose_seq[-1].position.y = -1.25
    # Here we use geometry_msgs/Quaternion z field to store heading
    start_end_pose.pose_seq[-1].orientation.z = 0.0

    r = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():

        pub.publish(start_end_pose)

        r.sleep()


if __name__ == '__main__':
    try:
        publish_start_end_pose_msg()
    except rospy.ROSInterruptException:
        pass
