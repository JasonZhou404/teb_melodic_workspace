/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <teb_local_planner/robot_footprint_model.h>

#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
RobotFootprintModelPtr robot_model;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;
double teb_total_time;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent &e);
void CB_publishCycle(const ros::TimerEvent &e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig &reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg);
Point2dContainer build_robot_model();
void set_via_points();

// =============== Main function =================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");

  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);

  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);

  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);

  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);

  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);

  // Setup manully set Via-points
  set_via_points();

  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));

  // Setup robot shape model
  robot_model = RobotFootprintModelPtr(new PolygonRobotFootprint(build_robot_model()));

  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

  // planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

  no_fixed_obstacles = obst_vector.size();

  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent &e)
{
  auto start = std::chrono::system_clock::now();
  planner->plan(PoseSE2(-6, 2.5, 0), PoseSE2(2.0, -1.25, 0)); // hardcoded start and goal for testing purposes
  std::chrono::duration<double> diff =
      std::chrono::system_clock::now() - start;
  teb_total_time = diff.count() * 1000;
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent &e)
{
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
  std::cout << "TEB total used time: " << teb_total_time << " ms.\n";
  ROS_INFO(("TEB total used time: " + std::to_string(teb_total_time) + " ms.").c_str());
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig &reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);

  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1)
    {
      if (obst_msg->obstacles.at(i).radius == 0)
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y)));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                               obst_msg->obstacles.at(i).polygon.points.front().y,
                                                               obst_msg->obstacles.at(i).radius)));
      }
    }
    else
    {
      PolygonObstacle *polyobst = new PolygonObstacle;
      for (size_t j = 0; j < obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex(obst_msg->obstacles.at(i).polygon.points[j].x,
                                 obst_msg->obstacles.at(i).polygon.points[j].y);
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }
    if (!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back(Eigen::Vector2d(point_msg->point.x, point_msg->point.y));
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint <= 0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  via_points.emplace_back(6.0, 2.5);
}

Point2dContainer build_robot_model()
{

  Point2dContainer footprint;
  // MKZ
  Eigen::Vector2d left_front_pt;
  left_front_pt.x() = 3.89;
  left_front_pt.y() = 1.055;
  footprint.push_back(left_front_pt);

  Eigen::Vector2d left_back_pt;
  left_back_pt.x() = -1.043;
  left_back_pt.y() = 1.055;
  footprint.push_back(left_back_pt);

  Eigen::Vector2d right_back_pt;
  right_back_pt.x() = -1.043;
  right_back_pt.y() = -1.055;
  footprint.push_back(right_back_pt);

  Eigen::Vector2d right_front_pt;
  right_front_pt.x() = 3.89;
  right_front_pt.y() = -1.055;
  footprint.push_back(right_front_pt);

  return footprint;
}

void set_via_points()
{
  ROS_INFO_ONCE("manually set Via-points. This message is printed once.");
  via_points.clear();
  // for starting point at [-6, 2.5, 0]
  via_points.emplace_back(0.0, 1);
  via_points.emplace_back(6.0, 2.0);
}
