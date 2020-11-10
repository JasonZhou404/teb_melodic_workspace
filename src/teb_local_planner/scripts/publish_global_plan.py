#!/usr/bin/env python

import os
import rospy
import csv
import sys
import time
from teb_local_planner.msg import PoseSeqMsg
from geometry_msgs.msg import Pose

DATA_PATH = '/home/jinyun/dev/teb_melodic_workspace/src/global_planner/data/hybrid_star_results/'


def list_files_with_suffix(dir_path, suffix):
    """List all sub-files with suffix in given dir_path."""
    return [os.path.join(root, f) for root, _, files
            in os.walk(dir_path) for f in files if f.endswith(suffix)]


def read_global_plan_from_files(folder):
    files = list_files_with_suffix(folder, '.txt')
    x_files = list(filter(lambda path: 'x.txt' in path, files))
    y_files = list(filter(lambda path: 'y.txt' in path, files))
    heading_files = list(filter(lambda path: 'phi.txt' in path, files))

    global_plan = {}
    for x in range(-8, 8):
        for y in [2, 2.5, 3, 3.5]:
            x_file = list(filter(lambda path: "start_x_{:.1f}".format(
                x) in path and "start_y_{:.1f}".format(y) in path, x_files))[0]
            y_file = list(filter(lambda path: "start_x_{:.1f}".format(
                x) in path and "start_y_{:.1f}".format(y) in path, y_files))[0]
            heading_file = list(filter(lambda path: "start_x_{:.1f}".format(
                x) in path and "start_y_{:.1f}".format(y) in path, heading_files))[0]

            global_plan["x_{:.1f}_y_{:.1f}".format(x, y)] = {
                'x': [], 'y': [], 'heading': []}

            with open(x_file, 'r') as read_file:
                lines = list(csv.reader(read_file))
                for row in lines:
                    global_plan["x_{:.1f}_y_{:.1f}".format(
                        x, y)]['x'].append(float(row[0]))

            with open(y_file, 'r') as read_file:
                lines = list(csv.reader(read_file))
                for row in lines:
                    global_plan["x_{:.1f}_y_{:.1f}".format(
                        x, y)]['y'].append(float(row[0]))

            with open(heading_file, 'r') as read_file:
                lines = list(csv.reader(read_file))
                for row in lines:
                    global_plan["x_{:.1f}_y_{:.1f}".format(
                        x, y)]['heading'].append(float(row[0]))
    print("There are {} global plans to be tested".format(len(global_plan)))
    return global_plan


def publish_global_plan_msg(global_plan):
    pub = rospy.Publisher('/test_optim_node/test_global_plan',
                          PoseSeqMsg, queue_size=10)
    rospy.init_node("test_global_plan")

    global_plan_idx = 0

    r = rospy.Rate(0.5)  # 0.2hz

    while pub.get_num_connections() == 0:
        print('waiting for subscriber')
        time.sleep(1)

    print("{} subscriber is ready".format(pub.get_num_connections()))

    while not rospy.is_shutdown():
        if global_plan_idx >= len(global_plan):
            print('all global plan published')
            # It's a hack here to republish the last global plan msg, as sometimes, the last plan feedback is
            # somehow dropped
            global_plan_idx = len(global_plan) - 1

        test_global_plan = PoseSeqMsg()
        test_global_plan.header.stamp = rospy.Time.now()
        test_global_plan.header.frame_id = "odom/map"  # CHANGE HERE: odom/map

        key_to_publish = list(global_plan.keys())[global_plan_idx]
        plan = global_plan[key_to_publish]

        for i in range(len(plan['x'])):
            test_global_plan.pose_seq.append(Pose())
            test_global_plan.pose_seq[-1].position.x = plan['x'][i]
            test_global_plan.pose_seq[-1].position.y = plan['y'][i]
            # Here we use geometry_msgs/Quaternion z field to store heading
            test_global_plan.pose_seq[-1].orientation.z = plan['heading'][i]

        test_global_plan.start_pose = key_to_publish

        pub.publish(test_global_plan)

        global_plan_idx += 1

        print(key_to_publish + " is published")

        r.sleep()


if __name__ == '__main__':
    global_plan = read_global_plan_from_files(DATA_PATH)

    try:
        publish_global_plan_msg(global_plan)
    except rospy.ROSInterruptException:
        pass
