#include "apf_local_planner/apf_local_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/costmap_2d.h>
#include <algorithm>
#include <cmath>

namespace apf_local_planner {

APFLocalPlannerROS::APFLocalPlannerROS()
  : initialized_(false),
    costmap_ros_(nullptr),
    tf_buffer_(nullptr),
    stuck_counter_(0),
    fallback_until_(0),
    dwa_planner_(nullptr)
{}

void APFLocalPlannerROS::initialize(std::string name,
                                    tf2_ros::Buffer* tf,
                                    costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) return;
  ros::NodeHandle pnh("~/" + name);

  tf_buffer_   = tf;
  costmap_ros_ = costmap_ros;

  // APF gains
  pnh.param("k_att",          k_att_,           1.0);
  pnh.param("k_rep",          k_rep_,         100.0);
  pnh.param("influence_dist", influence_dist_,  1.0);
  pnh.param("robot_radius",   robot_radius_,    0.2);
  pnh.param("max_linear_vel", max_linear_vel_,  0.3);
  pnh.param("max_angular_vel",max_angular_vel_, 1.0);

  // Visualization params
  double marker_rate;
  pnh.param("vector_field_rate", marker_rate,     2.0);
  marker_period_    = 1.0 / marker_rate;
  last_marker_time_ = ros::Time(0);
  pnh.param("vector_field_skip", grid_skip_,      4);
  pnh.param("arrow_length_mult", arrow_length_mult_, 1.2);
  pnh.param("arrow_shaft_diam",  arrow_shaft_diam_,  0.05);
  pnh.param("arrow_head_diam",   arrow_head_diam_,   0.05);
  marker_array_pub_ = pnh.advertise<visualization_msgs::MarkerArray>(
    "vector_field", 1);

  // Stuck detection & DWA fallback
  pnh.param("use_dwa_fallback",   use_dwa_fallback_,   false);
  pnh.param("stuck_dist_thresh",  stuck_dist_thresh_,  0.01);
  pnh.param("stuck_reset_iters",  stuck_reset_iters_,  20);
  pnh.param("fallback_duration",  fallback_duration_,  5.0);

  // init pose for stuck detection
  {
    geometry_msgs::PoseStamped rp;
    costmap_ros_->getRobotPose(rp);
    last_x_ = rp.pose.position.x;
    last_y_ = rp.pose.position.y;
  }

  // create DWA planner if fallback enabled
  if (use_dwa_fallback_) {
    dwa_planner_ = new dwa_local_planner::DWAPlannerROS();
    dwa_planner_->initialize(name + "/dwa", tf_buffer_, costmap_ros_);
  }

  initialized_ = true;
}

bool APFLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) return false;
  global_plan_ = plan;
  if (use_dwa_fallback_) {
    dwa_planner_->setPlan(plan);
  }
  return true;
}

bool APFLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_ || global_plan_.empty())
    return false;

  // 1) current robot pose
  geometry_msgs::PoseStamped rp;
  costmap_ros_->getRobotPose(rp);
  double rx  = rp.pose.position.x,
         ry  = rp.pose.position.y,
         yaw = tf2::getYaw(rp.pose.orientation);

  // 2) stuck detection
  double dx = rx - last_x_, dy = ry - last_y_;
  double moved = std::hypot(dx, dy);
  last_x_ = rx; last_y_ = ry;
  if (moved < stuck_dist_thresh_) {
    stuck_counter_++;
  } else {
    stuck_counter_ = 0;
  }

  // 3) if stuck long enough, enter fallback
  ros::Time now = ros::Time::now();
  if (use_dwa_fallback_ &&
      stuck_counter_ >= stuck_reset_iters_)
  {
    fallback_until_ = now + ros::Duration(fallback_duration_);
    stuck_counter_ = 0;
    ROS_WARN("APF: local minimum detected, fallback to DWA for %.1f s",
             fallback_duration_);
  }

  // 4) if in fallback window, call DWA
  if (use_dwa_fallback_ && now < fallback_until_) {
    return dwa_planner_->computeVelocityCommands(cmd_vel);
  }

  // 5) otherwise, run APF
  double fx_tot = 0.0, fy_tot = 0.0;
  computeForces(rx, ry, fx_tot, fy_tot);

  // 6) visualization
  if ((now - last_marker_time_).toSec() >= marker_period_) {
    publishVectorFieldMarkers();
    last_marker_time_ = now;
  }

  // 7) convert force → cmd_vel
  double mag   = std::hypot(fx_tot, fy_tot);
  double theta = std::atan2(fy_tot, fx_tot);
  double da    = angles::shortest_angular_distance(yaw, theta);

  cmd_vel.angular.z = std::max(-max_angular_vel_,
                              std::min(max_angular_vel_, da));

  double v = mag;
  if (std::fabs(da) > M_PI/4.0) v *= 0.3;
  cmd_vel.linear.x = std::max(0.0,
                              std::min(max_linear_vel_, v));

  return true;
}

bool APFLocalPlannerROS::isGoalReached()
{
  if (!initialized_ || global_plan_.empty())
    return false;
  geometry_msgs::PoseStamped rp;
  costmap_ros_->getRobotPose(rp);
  auto goal = global_plan_.back();
  double dx = goal.pose.position.x - rp.pose.position.x;
  double dy = goal.pose.position.y - rp.pose.position.y;
  return (std::hypot(dx, dy) < robot_radius_);
}

void APFLocalPlannerROS::computeForces(double wx, double wy,
                                       double& fx, double& fy)
{
  // Attractive
  auto goal = global_plan_.back();
  double gx = goal.pose.position.x,
         gy = goal.pose.position.y;
  fx += -k_att_ * (wx - gx);
  fy += -k_att_ * (wy - gy);

  // Repulsive from obstacles within influence_dist_
  costmap_2d::Costmap2D* cmap = costmap_ros_->getCostmap();
  double res = cmap->getResolution(), inf = influence_dist_;
  for (double ox = wx - inf; ox <= wx + inf; ox += res) {
    for (double oy = wy - inf; oy <= wy + inf; oy += res) {
      unsigned int mx,my;
      if (!cmap->worldToMap(ox, oy, mx, my)) continue;
      if (cmap->getCost(mx,my) < costmap_2d::LETHAL_OBSTACLE) continue;
      double dx = wx - ox, dy = wy - oy, d = std::hypot(dx, dy);
      if (d < 1e-3 || d > inf) continue;
      double m = k_rep_ * (1.0/d - 1.0/inf) / (d*d);
      fx += m * (dx/d);
      fy += m * (dy/d);
    }
  }
}

void APFLocalPlannerROS::publishVectorFieldMarkers()
{
  visualization_msgs::MarkerArray arr;
  auto* cmap = costmap_ros_->getCostmap();
  double res = cmap->getResolution(),
         ox  = cmap->getOriginX(),
         oy  = cmap->getOriginY();
  unsigned int nx = cmap->getSizeInCellsX(),
               ny = cmap->getSizeInCellsY();
  int id = 0;

  // final goal
  auto goal = global_plan_.back();
  double gx = goal.pose.position.x,
         gy = goal.pose.position.y;

  for (unsigned int i = 0; i < nx; i += grid_skip_) {
    for (unsigned int j = 0; j < ny; j += grid_skip_) {
      double wx = ox + (i + 0.5)*res;
      double wy = oy + (j + 0.5)*res;
      unsigned char cost = cmap->getCost(i,j);

      // Attractive (green)
      {
        double fx_att = -k_att_ * (wx - gx);
        double fy_att = -k_att_ * (wy - gy);
        double mag_att = std::hypot(fx_att, fy_att);
        if (mag_att > 1e-6) {
          fx_att = fx_att/mag_att * res * arrow_length_mult_;
          fy_att = fy_att/mag_att * res * arrow_length_mult_;
          visualization_msgs::Marker mk;
          mk.header.frame_id = costmap_ros_->getGlobalFrameID();
          mk.header.stamp    = ros::Time::now();
          mk.ns              = "apf_attr";
          mk.id              = id++;
          mk.type            = visualization_msgs::Marker::ARROW;
          mk.action          = visualization_msgs::Marker::ADD;
          mk.scale.x         = arrow_length_mult_ * res;
          mk.scale.y         = arrow_shaft_diam_;
          mk.scale.z         = arrow_head_diam_;
          mk.color.r         = 0.0;
          mk.color.g         = 1.0;
          mk.color.a         = 1.0;
          mk.pose.orientation.w = 1.0;
          geometry_msgs::Point ps, pe;
          ps.x = wx;        ps.y = wy;        ps.z = 0.02;
          pe.x = wx + fx_att; pe.y = wy + fy_att; pe.z = 0.02;
          mk.points = {ps, pe};
          mk.lifetime = ros::Duration(marker_period_);
          arr.markers.push_back(mk);
        }
      }

      // Repulsive (red)
      {
        double fx_tot=0.0, fy_tot=0.0;
        computeForces(wx, wy, fx_tot, fy_tot);
        double fx_att = -k_att_ * (wx - gx);
        double fy_att = -k_att_ * (wy - gy);
        double fx_rep = fx_tot - fx_att;
        double fy_rep = fy_tot - fy_att;
        double mag_rep = std::hypot(fx_rep, fy_rep);
        if (mag_rep > 1e-6) {
          fx_rep = fx_rep/mag_rep * res * arrow_length_mult_;
          fy_rep = fy_rep/mag_rep * res * arrow_length_mult_;
          visualization_msgs::Marker mk;
          mk.header.frame_id = costmap_ros_->getGlobalFrameID();
          mk.header.stamp    = ros::Time::now();
          mk.ns              = "apf_rep";
          mk.id              = id++;
          mk.type            = visualization_msgs::Marker::ARROW;
          mk.action          = visualization_msgs::Marker::ADD;
          mk.scale.x         = arrow_length_mult_ * res;
          mk.scale.y         = arrow_shaft_diam_;
          mk.scale.z         = arrow_head_diam_;
          mk.color.r         = 1.0;
          mk.color.g         = 0.0;
          mk.color.a         = 1.0;
          mk.pose.orientation.w = 1.0;
          geometry_msgs::Point ps, pe;
          ps.x = wx;        ps.y = wy;        ps.z = 0.04;
          pe.x = wx + fx_rep; pe.y = wy + fy_rep; pe.z = 0.04;
          mk.points = {ps, pe};
          mk.lifetime = ros::Duration(marker_period_);
          arr.markers.push_back(mk);
        }
      }
    }
  }

  marker_array_pub_.publish(arr);
}

}  // namespace apf_local_planner

PLUGINLIB_EXPORT_CLASS(
  apf_local_planner::APFLocalPlannerROS,
  nav_core::BaseLocalPlanner)
