#!/usr/bin/env python

import rospy
import math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32


def publish_obstacle_msg():
    pub = rospy.Publisher('/test_optim_node/obstacles',
                          ObstacleArrayMsg, queue_size=1)
    rospy.init_node("obstacle_msg")

    obstacle_msg = ObstacleArrayMsg()
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "odom/map"  # CHANGE HERE: odom/map

    # Add bottom obstacle
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[0].id = 0
    v0 = Point32()
    v0.x = -12.0
    v0.y = 0.0
    v1 = Point32()
    v1.x = 0.0
    v1.y = 0.0
    v2 = Point32()
    v2.x = 0.0
    v2.y = -2.5
    v3 = Point32()
    v3.x = 7.5
    v3.y = -2.5
    v4 = Point32()
    v4.x = 7.5
    v4.y = 0
    v5 = Point32()
    v5.x = 17
    v5.y = 0
    v6 = Point32()
    v6.x = 17
    v6.y = -10
    v7 = Point32()
    v7.x = -12
    v7.y = -10
    obstacle_msg.obstacles[0].polygon.points = [v0, v1, v2, v3, v4, v5, v6, v7]

    # Add top obstacle
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[1].id = 1
    v0 = Point32()
    v0.x = 17.0
    v0.y = 5.6
    v1 = Point32()
    v1.x = -12.0
    v1.y = 5.6
    v2 = Point32()
    v2.x = -12.0
    v2.y = 10.0
    v3 = Point32()
    v3.x = 17.0
    v3.y = 10.0
    obstacle_msg.obstacles[1].polygon.points = [v0, v1, v2, v3]

    r = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():

        pub.publish(obstacle_msg)

        r.sleep()


if __name__ == '__main__':
    try:
        publish_obstacle_msg()
    except rospy.ROSInterruptException:
        pass
