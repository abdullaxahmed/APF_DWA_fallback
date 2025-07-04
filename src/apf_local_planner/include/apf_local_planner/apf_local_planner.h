#ifndef APF_LOCAL_PLANNER_H
#define APF_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <dwa_local_planner/dwa_planner_ros.h>

namespace apf_local_planner {

class APFLocalPlannerROS : public nav_core::BaseLocalPlanner {
public:
  APFLocalPlannerROS();
  ~APFLocalPlannerROS() override {}

  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  // ─── Dependencies & State ────────────────────────────────────────
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer*          tf_buffer_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  // ─── APF Gains & Limits ─────────────────────────────────────────
  double k_att_, k_rep_, influence_dist_;
  double robot_radius_, max_linear_vel_, max_angular_vel_;

  // ─── Visualization ─────────────────────────────────────────────
  ros::Publisher            marker_array_pub_;
  double                    marker_period_;
  ros::Time                 last_marker_time_;
  int                       grid_skip_;
  double                    arrow_length_mult_, arrow_shaft_diam_, arrow_head_diam_;

  // ─── Stuck Detection & Fallback ─────────────────────────────────
  bool                      use_dwa_fallback_;
  double                    stuck_dist_thresh_;
  int                       stuck_reset_iters_, stuck_counter_;
  double                    fallback_duration_;    // seconds
  ros::Time                 fallback_until_;
  dwa_local_planner::DWAPlannerROS* dwa_planner_;

  double last_x_, last_y_;

  // ─── Core APF ──────────────────────────────────────────────────
  void computeForces(double wx, double wy, double& fx, double& fy);

  // ─── Visualization Helper ─────────────────────────────────────
  void publishVectorFieldMarkers();
};

}  // namespace apf_local_planner

#endif  // APF_LOCAL_PLANNER_H
