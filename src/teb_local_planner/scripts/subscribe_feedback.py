#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the info.

import csv
import rospy
import math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import matplotlib.pyplot as plotter
from shapely.geometry import Polygon, LineString, Point

MKZ_FRONT_EDGE_TO_CENTER = 3.89
MKZ_BACK_EDGE_TO_CENTER = 1.043
MKZ_WIDTH = 2.11

RESULTS_TABLE_CSV = '/home/jinyun/dev/teb_melodic_workspace/src/teb_local_planner/scripts/test_results.csv'


def create_off_centered_box(rear_x, rear_y, heading, width=MKZ_WIDTH,
                            front_edge_to_center=MKZ_FRONT_EDGE_TO_CENTER,
                            back_edge_to_center=MKZ_BACK_EDGE_TO_CENTER):
    offset = 0.0
    length = front_edge_to_center + back_edge_to_center
    if front_edge_to_center > back_edge_to_center:
        offset = front_edge_to_center - length / 2.0
    else:
        offset = back_edge_to_center - length / 2.0
    center_x = rear_x + offset * math.cos(heading)
    center_y = rear_y + offset * math.sin(heading)

    dx1 = math.cos(heading) * length / 2.0
    dy1 = math.sin(heading) * length / 2.0
    dx2 = math.sin(heading) * width / 2.0
    dy2 = -math.cos(heading) * width / 2.0
    return Polygon([[center_x + dx1 + dx2, center_y + dy1 + dy2],
                    [center_x + dx1 - dx2, center_y + dy1 - dy2],
                    [center_x - dx1 - dx2, center_y - dy1 - dy2],
                    [center_x - dx1 + dx2, center_y - dy1 + dy2]])


def create_parking_boundaries():
    upper_boundaries = Polygon([[17, 5], [-10, 5], [-10, 10], [17, 10]])
    lower_boundaries = Polygon(
        [[-10, 0], [0, 0], [0, -2.5], [7, -2.5], [7, 0], [17, 0], [17, -10], [-10, -10]])
    return upper_boundaries, lower_boundaries


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


def feedback_callback(data):
    global trajectory
    global time
    global cur_start_pose_id
    if not data.trajectories:  # empty
        trajectory = []
        time = None
        return
    trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
    time = data.computation_time
    cur_start_pose_id = data.start_pose


def plot_velocity_profile(fig, ax_v, t, v):
    ax_v.cla()
    ax_v.grid()
    ax_v.set_ylabel('Trans. velocity [m/s]')
    ax_v.set_xlabel('Time [s]')
    ax_v.plot(t, v, '-bx')
    fig.canvas.draw()


def plot_curvature_profile(fig, ax_curvature, s, curvature):
    ax_curvature.cla()
    ax_curvature.grid()
    ax_curvature.set_ylabel('curvature [m^-1]')
    ax_curvature.set_xlabel('accumulated_s [m]')
    ax_curvature.plot(s, curvature, '-bx')
    fig.canvas.draw()


def calc_point_distance(x0, y0, x1, y1):
    return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)


def dump_statistics():
    print(timing_table)
    print(collision_table)
    print(curvature_invalid_table)
    success_table = [not a and not b for a, b in zip(
        collision_table, curvature_invalid_table)]
    with open(RESULTS_TABLE_CSV, 'w') as csvfile:
        csv.writer(csvfile).writerow(['start_pose_id',
                                      'average time is {}'.format(
                                          sum(timing_table)/len(timing_table)),
                                      'collision rate is {}'.format(
                                          sum(collision_table)/len(collision_table)),
                                      'curvature invalid rate is {}'.format(
                                          sum(curvature_invalid_table)/len(curvature_invalid_table)),
                                      'success rate is {}'.format(sum(success_table)/len(success_table))])
        for i in range(len(timing_table)):
            csv.writer(csvfile).writerow(
                [start_pose_id_table[i],
                 timing_table[i],
                 collision_table[i],
                 curvature_invalid_table[i],
                 success_table[i]])


def feedback_subscriber():
    global trajectory
    global timing_table
    global collision_table
    global start_pose_id_table
    global curvature_invalid_table

    rospy.init_node("test_feedback_subscriber", anonymous=True)

    topic_name = "/test_optim_node/teb_feedback"
    topic_name = rospy.get_param('~feedback_topic', topic_name)
    rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback,
                     queue_size=10)  # define feedback topic here!

    rospy.loginfo(
        "Feedback published on '%s'.", topic_name)
    rospy.loginfo(
        "Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

    rospy.on_shutdown(dump_statistics)

    fig, (ax_v, ax_curvature) = plotter.subplots(2)
    plotter.subplots_adjust(hspace=0.5)

    plotter.ion()
    plotter.show()

    upper_boundaries, lower_boundaries = create_parking_boundaries()

    r = rospy.Rate(10)  # define rate here
    last_start_pose_id = None
    while not rospy.is_shutdown():

        if len(trajectory) == 0:
            continue

        if cur_start_pose_id == last_start_pose_id or len(cur_start_pose_id) == 0:
            continue
        last_start_pose_id = cur_start_pose_id

        print(cur_start_pose_id + " is received")
        
        t = []
        v = []
        omega = []
        s = []
        curvature = []

        pre_pose = [trajectory[0].pose.position.x,
                    trajectory[0].pose.position.y]
        accumulated_s = 0
        for point in trajectory:
            t.append(point.time_from_start.to_sec())
            v.append(point.velocity.linear.x)
            omega.append(point.velocity.angular.z)
            if abs(v[-1]) < 0.1:
                instant_curvature = "NAN"
            else:
                instant_curvature = omega[-1] / v[-1]
            curvature.append(instant_curvature)
            accumulated_s += calc_point_distance(
                pre_pose[0], pre_pose[1], point.pose.position.x, point.pose.position.y)
            s.append(accumulated_s)
            pre_pose = [point.pose.position.x, point.pose.position.y]

        for i in range(len(curvature)):
            if curvature[i] == "NAN":
                curvature[i] = curvature[i+1] if i + \
                    1 < len(curvature) else curvature[i-1]

        plot_velocity_profile(fig, ax_v, np.asarray(
            t), np.asarray(v))
        plot_curvature_profile(
            fig, ax_curvature, np.asarray(s), np.asarray(curvature))

        if time is not None:
            timing_table.append(time)

        collision_table.append(False)
        for i in range(len(trajectory)):
            x = trajectory[i].pose.orientation.x
            y = trajectory[i].pose.orientation.y
            z = trajectory[i].pose.orientation.z
            w = trajectory[i].pose.orientation.w
            heading = quaternion_to_euler(x, y, z, w)[0]
            ego_box = create_off_centered_box(trajectory[i].pose.position.x,
                                              trajectory[i].pose.position.y,
                                              heading)
            if ego_box.intersects(upper_boundaries) or ego_box.intersects(lower_boundaries):
                collision_table[-1] = True
                break

        start_pose_id_table.append(last_start_pose_id)

        curvature_invalid_table.append(False)
        for kappa in curvature:
            if instant_curvature != "NAN" and abs(kappa) > 0.2:
                curvature_invalid_table[-1] = True
                break
        r.sleep()


if __name__ == '__main__':
    try:
        trajectory = []
        time = None
        cur_start_pose_id = None
        timing_table = []
        collision_table = []
        start_pose_id_table = []
        curvature_invalid_table = []
        feedback_subscriber()
    except rospy.ROSInterruptException:
        pass
