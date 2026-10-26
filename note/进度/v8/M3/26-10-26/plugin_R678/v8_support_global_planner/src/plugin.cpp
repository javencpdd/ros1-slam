#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace v8_support_global_planner {
class SupportGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override {
    ros::NodeHandle private_nh("~" + name);
    private_nh.param<std::string>("service", service_, "/make_plan");
    ros::NodeHandle nh;
    client_ = nh.serviceClient<nav_msgs::GetPlan>(service_);
  }
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    plan.clear();
    if (!client_.exists()) { ROS_ERROR("[v8 planner] service %s not available", service_.c_str()); return false; }
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 0.1;
    if (!client_.call(srv)) { ROS_ERROR("[v8 planner] service call failed"); return false; }
    if (srv.response.plan.poses.empty()) { ROS_ERROR("[v8 planner] planner refused or returned no path"); return false; }
    plan = srv.response.plan.poses;
    ROS_INFO("[v8 planner] delegated plan: %zu poses", plan.size());
    return true;
  }
 private:
  std::string service_;
  ros::ServiceClient client_;
};
}  // namespace v8_support_global_planner
PLUGINLIB_EXPORT_CLASS(v8_support_global_planner::SupportGlobalPlanner, nav_core::BaseGlobalPlanner)
