import sys
base = '/work/scan_priv/src'

def patch(path, old, new, count=1):
    s = open(path).read()
    assert s.count(old) == count, (path, s.count(old), old[:60])
    open(path, 'w').write(s.replace(old, new))

gmh = base + '/plan_env/include/plan_env/grid_map.h'
patch(gmh, "  inline double getResolution();",
      "  inline double getResolution();\n\n  void setLatestCloudStamp(const ros::Time& t) { latest_cloud_stamp_ = t; }\n  ros::Time getLatestCloudStamp() const { return latest_cloud_stamp_; }")
patch(gmh, "class GridMap {\npublic:", "class GridMap {\npublic:\n  ros::Time latest_cloud_stamp_;")

gmc = base + '/plan_env/src/grid_map.cpp'
patch(gmc, """  if (!md_.has_ray_pose_)
  {
    ROS_WARN_THROTTLE(1.0, "[GridMap] no /grid_map/sensor_pose yet for lidar cloud update.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;""", """  if (!md_.has_ray_pose_)
  {
    ROS_WARN_THROTTLE(1.0, "[GridMap] no /grid_map/sensor_pose yet for lidar cloud update.");
    return;
  }

  latest_cloud_stamp_ = img->header.stamp;

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;""")

fsh = base + '/plan_manage/include/plan_manage/scan_replan_fsm.h'
patch(fsh, "#include <plan_env/grid_map.h>", "#include <plan_env/grid_map.h>\n#include <std_msgs/Float64MultiArray.h>")
patch(fsh, "    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_;",
      "    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_, plan_timing_pub_;")

fsc = base + '/plan_manage/src/scan_replan_fsm.cpp'
patch(fsc, '    self_inflation_pub_ = nh.advertise<visualization_msgs::Marker>("self_inflation", 10, true);',
      '    self_inflation_pub_ = nh.advertise<visualization_msgs::Marker>("self_inflation", 10, true);\n    plan_timing_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/debug/plan_timing", 50);')
patch(fsc, """  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();""", """  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    ros::Time plan_start = ros::Time::now();
    ros::Time cloud_stamp = planner_manager_->grid_map_->getLatestCloudStamp();

    getLocalTarget();""")
patch(fsc, """      bspline_pub_.publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);""", """      bspline_pub_.publish(bspline);

      std_msgs::Float64MultiArray tm;
      tm.data = {cloud_stamp.toSec(), plan_start.toSec(), ros::Time::now().toSec(), (double)bspline.traj_id};
      plan_timing_pub_.publish(tm);

      visualization_->displayOptimalTraj(info->position_traj_, 0);""")
print('patched')
