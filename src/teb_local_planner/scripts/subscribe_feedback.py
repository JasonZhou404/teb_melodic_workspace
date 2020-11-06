#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the info.

import rospy
import math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import matplotlib.pyplot as plotter


def feedback_callback(data):
    global trajectory

    if not data.trajectories:  # empty
        trajectory = []
        return
    trajectory = data.trajectories[data.selected_trajectory_idx].trajectory


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


def velocity_plotter():
    global trajectory
    rospy.init_node("visualize_velocity_profile", anonymous=True)

    topic_name = "/test_optim_node/teb_feedback"
    topic_name = rospy.get_param('~feedback_topic', topic_name)
    rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback,
                     queue_size=1)  # define feedback topic here!

    rospy.loginfo(
        "Visualizing velocity profile published on '%s'.", topic_name)
    rospy.loginfo(
        "Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

    fig, (ax_v, ax_curvature) = plotter.subplots(2)
    plotter.subplots_adjust(hspace=0.5)

    plotter.ion()
    plotter.show()

    r = rospy.Rate(2)  # define rate here
    while not rospy.is_shutdown():

        if len(trajectory) == 0:
            continue

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
        plot_curvature_profile(fig, ax_curvature, np.asarray(s), np.asarray(curvature))

        r.sleep()


if __name__ == '__main__':
    try:
        trajectory = []
        velocity_plotter()
    except rospy.ROSInterruptException:
        pass
