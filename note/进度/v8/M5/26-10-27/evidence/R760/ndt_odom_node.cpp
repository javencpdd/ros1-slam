#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pclomp/ndt_omp.h>
#include <Eigen/Geometry>
#include <mutex>

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

static Eigen::Matrix4f makeT(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Affine3f a = Eigen::Affine3f::Identity();
  a.translation() << x, y, z;
  a.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  a.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
  a.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
  return a.matrix();
}

class NdtOdom {
 public:
  NdtOdom() : nh_("~") {
    std::string map_path;
    nh_.param<std::string>("map_path", map_path, "/work/maps/m1_world.pcd");
    nh_.param<double>("resolution", resolution_, 1.0);
    nh_.param<double>("score_max", score_max_, 2.0);
    nh_.param<double>("ext_x", ext_x_, 0.0);
    nh_.param<double>("ext_y", ext_y_, 0.0);
    nh_.param<double>("ext_z", ext_z_, 0.34);
    nh_.param<double>("ext_pitch", ext_pitch_, 0.2617993878);
    nh_.param<bool>("global_search", global_search_, false);
    nh_.param<double>("gs_x_min", gs_x_min_, -1.0);
    nh_.param<double>("gs_x_max", gs_x_max_, 5.0);
    nh_.param<double>("gs_y_min", gs_y_min_, -1.0);
    nh_.param<double>("gs_y_max", gs_y_max_, 1.0);
    nh_.param<double>("gs_xy_step", gs_xy_step_, 0.5);
    nh_.param<double>("gs_yaw_step", gs_yaw_step_, 1.0);

    Cloud::Ptr map(new Cloud);
    if (pcl::io::loadPCDFile<PointT>(map_path, *map) < 0 || map->empty()) {
      ROS_FATAL("[v8_ndt] cannot load map %s", map_path.c_str());
      ros::shutdown();
      return;
    }
    target_.reset(new Cloud);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setInputCloud(map);
    vg.filter(*target_);

    ndt_.setResolution(resolution_);
    ndt_.setInputTarget(target_);
    ndt_.setMaximumIterations(30);
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.1);
    ndt_.setNumThreads(4);

    t_base_lidar_ = makeT(ext_x_, ext_y_, ext_z_, 0.0, ext_pitch_, 0.0);
    t_lidar_base_ = t_base_lidar_.inverse();
    t_base_map_ = Eigen::Matrix4f::Identity();
    have_pose_ = false;

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ndt_odom", 10);
    score_pub_ = nh_.advertise<std_msgs::Float32>("/ndt_score", 10);
    dbg_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/ndt_dbg", 10);
    sub_ = nh_.subscribe("/rslidar_points", 1, &NdtOdom::cloudCb, this);
    ROS_INFO("[v8_ndt] map=%s points=%zu res=%.2f score_max=%.2f", map_path.c_str(),
             target_->size(), resolution_, score_max_);
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (cloud->size() < 100) return;
    Cloud::Ptr ds(new Cloud);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setInputCloud(cloud);
    vg.filter(*ds);

    ndt_.setInputSource(ds);
    if (!have_pose_ && global_search_) {
      Cloud tmp;
      Eigen::Matrix4f best = Eigen::Matrix4f::Identity();
      double best_score = 1e9;
      for (double gx = gs_x_min_; gx <= gs_x_max_ + 1e-9; gx += gs_xy_step_) {
        for (double gy = gs_y_min_; gy <= gs_y_max_ + 1e-9; gy += gs_xy_step_) {
          for (double yaw = -M_PI; yaw < M_PI; yaw += gs_yaw_step_) {
            Eigen::Matrix4f cand = makeT(gx, gy, 0.0, 0.0, 0.0, yaw) * t_base_lidar_;
            ndt_.align(tmp, cand);
            if (!ndt_.hasConverged()) continue;
            double sc = ndt_.getFitnessScore();
            if (sc < best_score) { best_score = sc; best = ndt_.getFinalTransformation() * t_lidar_base_; }
          }
        }
      }
      if (best_score < 1e8) {
        t_base_map_ = best; have_pose_ = true;
        ROS_INFO("[v8_ndt] global search best score=%.4f", best_score);
      }
    }
    Eigen::Matrix4f guess = t_base_map_ * t_base_lidar_;
    if (have_prev_) {
      Eigen::Matrix4f dT = prev_base_map_.inverse() * t_base_map_;
      float dyaw = atan2(dT(1, 0), dT(0, 0));
      Eigen::Matrix4f step = Eigen::Matrix4f::Identity();
      step(0, 0) = cos(dyaw); step(0, 1) = -sin(dyaw);
      step(1, 0) = sin(dyaw); step(1, 1) = cos(dyaw);
      step(0, 3) = dT(0, 3); step(1, 3) = dT(1, 3);
      // R757：高偏航帧（转弯）不外推平移，避免把 NDT 噪声当速度、把初值推离走廊。
      if (fabs(dyaw) > 0.05f) { step(0, 3) = 0.0f; step(1, 3) = 0.0f; }
      guess = (t_base_map_ * step) * t_base_lidar_;
    }
    Cloud out;
    ndt_.align(out, guess);
    Eigen::Matrix4f t_lidar_map = ndt_.getFinalTransformation();
    double score = ndt_.getFitnessScore();
    std_msgs::Float32 s; s.data = score; score_pub_.publish(s);
    Eigen::Matrix4f t_new = t_lidar_map * t_lidar_base_;
    bool accept_new = ndt_.hasConverged() && score < score_max_;
    if (accept_new) {
      Eigen::Matrix4f rel = t_base_map_.inverse() * t_new;
      const float jt = rel.block<3, 1>(0, 3).norm();
      const float jy = fabs(atan2(rel(1, 0), rel(0, 0)));
      const bool jump = jt > 0.35f || jy > 0.35f;
      if (jump) {
        // R760：单帧不可信跳变直接拒绝；连续 20 帧仍被拒才允许重锁（约 2.5 s）。
        reject_streak_++;
        if (reject_streak_ >= 20) { accept_new = true; reject_streak_ = 0; }
        else accept_new = false;
      } else {
        reject_streak_ = 0;
      }
    }
    const bool accepted = accept_new;
    std_msgs::Float32MultiArray dbg;
    dbg.data = {accepted ? 1.0f : 0.0f, (float)score, ndt_.hasConverged() ? 1.0f : 0.0f};
    dbg_pub_.publish(dbg);
    if (accepted) {
      t_base_map_ = t_lidar_map * t_lidar_base_;
      have_pose_ = true;
    } else if (!have_pose_) {
      t_base_map_ = Eigen::Matrix4f::Identity();
    }
    prev_base_map_ = t_base_map_;
    have_prev_ = true;
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    Eigen::Affine3f a(t_base_map_);
    Eigen::Quaternionf q(a.rotation());
    odom.pose.pose.position.x = a.translation().x();
    odom.pose.pose.position.y = a.translation().y();
    odom.pose.pose.position.z = a.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_.publish(odom);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher odom_pub_, score_pub_, dbg_pub_;
  pclomp::NormalDistributionsTransform<PointT, PointT> ndt_;
  Cloud::Ptr target_;
  Eigen::Matrix4f t_base_lidar_, t_lidar_base_, t_base_map_;
  bool have_pose_, global_search_, have_prev_ = false;
  Eigen::Matrix4f prev_base_map_;
  int reject_streak_ = 0;
  double resolution_, score_max_, ext_x_, ext_y_, ext_z_, ext_pitch_;
  double gs_x_min_, gs_x_max_, gs_y_min_, gs_y_max_, gs_xy_step_, gs_yaw_step_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "v8_ndt_odom");
  NdtOdom node;
  ros::spin();
  return 0;
}
