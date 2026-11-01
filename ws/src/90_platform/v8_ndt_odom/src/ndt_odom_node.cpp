#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pclomp/ndt_omp.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <limits>
#include <vector>
#include <v8_ndt_odom/pose_history.h>

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
    std::string role;
    nh_.param<std::string>("output_role", role, "monitor");
    if (role != "monitor") {
      ROS_FATAL("[v8_ndt] only monitor role is authorized; continuous backend is not enabled");
      ros::shutdown();
      return;
    }
    if (!nh_.getParam("map_path", map_path) || map_path.empty()) {
      ROS_FATAL("[v8_ndt] map_path must be explicit");
      ros::shutdown();
      return;
    }
    nh_.param<double>("resolution", resolution_, 1.0);
    nh_.param<double>("score_max", score_max_, 2.0);
    nh_.param<double>("ext_x", ext_x_, 0.0);
    nh_.param<double>("ext_y", ext_y_, 0.0);
    nh_.param<double>("ext_z", ext_z_, 0.34);
    nh_.param<double>("ext_pitch", ext_pitch_, 0.2617993878);
    nh_.param<bool>("global_search", global_search_, false);
    nh_.param<bool>("use_prediction", use_prediction_, true);
    nh_.param<int>("max_iterations", max_iterations_, 30);
    nh_.param<double>("step_size", step_size_, 0.1);
    nh_.param<double>("gs_x_min", gs_x_min_, -1.0);
    nh_.param<double>("gs_x_max", gs_x_max_, 5.0);
    nh_.param<double>("gs_y_min", gs_y_min_, -1.0);
    nh_.param<double>("gs_y_max", gs_y_max_, 1.0);
    nh_.param<double>("gs_xy_step", gs_xy_step_, 0.5);
    nh_.param<double>("gs_yaw_step", gs_yaw_step_, 1.0);
    nh_.param<double>("init_x", init_x_, 0.0);
    nh_.param<double>("init_y", init_y_, 0.0);
    nh_.param<double>("init_yaw_deg", init_yaw_deg_, 0.0);
    nh_.param<double>("relocalize_radius_m", relocalize_radius_m_, 0.5);
    nh_.param<double>("relocalize_xy_step_m", relocalize_xy_step_m_, 0.2);
    nh_.param<double>("relocalize_yaw_span_deg", relocalize_yaw_span_deg_, 10.0);
    nh_.param<double>("relocalize_yaw_step_deg", relocalize_yaw_step_deg_, 5.0);
    nh_.param<double>("relocalize_budget_s", relocalize_budget_s_, 8.0);
    nh_.param<double>("relocalize_score_max", relocalize_score_max_, -1.0);
    nh_.param<double>("relocalize_max_displacement_m", relocalize_max_displacement_m_, -1.0);
    if (relocalize_score_max_ <= 0.0) relocalize_score_max_ = score_max_;
    if (relocalize_max_displacement_m_ <= 0.0) {
      relocalize_max_displacement_m_ = relocalize_radius_m_ + relocalize_xy_step_m_;
    }
    if (relocalize_radius_m_ < 0.0 || relocalize_xy_step_m_ <= 0.0
        || relocalize_yaw_span_deg_ < 0.0 || relocalize_yaw_step_deg_ <= 0.0
        || relocalize_budget_s_ <= 0.0) {
      ROS_FATAL("[v8_ndt] relocalize radius/step/span/budget must be finite and positive");
      ros::shutdown();
      return;
    }

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
    ndt_.setMaximumIterations(max_iterations_);
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(step_size_);
    ndt_.setNumThreads(4);

    t_base_lidar_ = makeT(ext_x_, ext_y_, ext_z_, 0.0, ext_pitch_, 0.0);
    t_lidar_base_ = t_base_lidar_.inverse();
    t_base_map_ = makeT(init_x_, init_y_, 0.0, 0.0, 0.0, init_yaw_deg_ * M_PI / 180.0);
    have_pose_ = (init_x_ != 0.0 || init_y_ != 0.0 || init_yaw_deg_ != 0.0);
    history_.reset(t_base_map_);
    if (have_pose_) ROS_INFO("[v8_ndt] initial pose seed (%.2f, %.2f, %.1f deg)", init_x_, init_y_, init_yaw_deg_);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    score_pub_ = nh_.advertise<std_msgs::Float32>("score", 10);
    dbg_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug", 10);
    health_pub_ = nh_.advertise<std_msgs::Bool>("healthy", 10, true);
    relocalized_pub_ = nh_.advertise<std_msgs::Bool>("relocalized", 10, true);
    publishHealth(false);
    publishRelocalized(false);
    reset_srv_ = nh_.advertiseService("reset", &NdtOdom::resetCb, this);
    relocalize_srv_ = nh_.advertiseService("relocalize", &NdtOdom::relocalizeCb, this);
    initialpose_sub_ = nh_.subscribe("initialpose", 1, &NdtOdom::initialPoseCb, this);
    std::string cloud_topic;
    nh_.param<std::string>("cloud_topic", cloud_topic, "/rslidar_points");
    sub_ = nh_.subscribe(cloud_topic, 1, &NdtOdom::cloudCb, this);
    ROS_INFO("[v8_ndt] map=%s points=%zu res=%.2f score_max=%.2f", map_path.c_str(),
             target_->size(), resolution_, score_max_);
  }

  void publishHealth(bool accepted) {
    std_msgs::Bool message;
    message.data = accepted;
    health_pub_.publish(message);
  }

  void publishRelocalized(bool accepted) {
    std_msgs::Bool message;
    message.data = accepted;
    relocalized_pub_.publish(message);
  }

  bool resetCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    t_base_map_ = makeT(init_x_, init_y_, 0.0, 0.0, 0.0, init_yaw_deg_ * M_PI / 180.0);
    history_.reset(t_base_map_);
    have_pose_ = false;
    accept_align_ = true;
    relocalize_requested_ = false;
    one_shot_relocalize_ = false;
    have_rej_pose_ = false;
    rej_consistent_ = 0;
    last_accept_score_ = 1e9;
    publishHealth(false);
    publishRelocalized(false);
    response.success = true;
    response.message = "monitor reset; next cloud uses configured seed/global search";
    return true;
  }

  bool relocalizeCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (relocalize_requested_) {
      response.success = false;
      response.message = "relocalize already armed";
      return true;
    }
    relocalize_requested_ = true;
    one_shot_relocalize_ = true;
    accept_align_ = false;  // No odom may escape until the bounded search succeeds.
    have_pose_ = false;
    have_rej_pose_ = false;
    rej_consistent_ = 0;
    history_.reset(t_base_map_);
    publishHealth(false);
    publishRelocalized(false);
    response.success = true;
    response.message = "armed; bounded local search around last monitor pose";
    return true;
  }

  void initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    const auto& position = msg->pose.pose.position;
    const auto& orientation = msg->pose.pose.orientation;
    const double norm = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y
                                  + orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)
        || !std::isfinite(norm) || norm < 1e-6) {
      ROS_WARN("[v8_ndt] rejected non-finite initialpose");
      return;
    }
    const double yaw = std::atan2(2.0 * (orientation.w * orientation.z
                                        + orientation.x * orientation.y),
                                  1.0 - 2.0 * (orientation.y * orientation.y
                                               + orientation.z * orientation.z));
    t_base_map_ = makeT(position.x, position.y, 0.0, 0.0, 0.0, yaw);
    history_.reset(t_base_map_);
    have_pose_ = true;
    accept_align_ = true;
    relocalize_requested_ = false;
    one_shot_relocalize_ = false;
    have_rej_pose_ = false;
    rej_consistent_ = 0;
    last_accept_score_ = 1e9;
    publishHealth(false);  // A seed is not an accepted observation.
    publishRelocalized(false);
    ROS_INFO("[v8_ndt] initialpose seed (%.3f, %.3f, %.1f deg)", position.x, position.y,
             yaw * 180.0 / M_PI);
  }

  bool boundedRelocalize() {
    // A relocalize request may only search near the last monitor pose.  Full-map
    // search (global_search) stays as the separate, deliberately unbounded path.
    const double center_x = t_base_map_(0, 3);
    const double center_y = t_base_map_(1, 3);
    const float center_yaw = std::atan2(t_base_map_(1, 0), t_base_map_(0, 0));
    const double yaw_span = relocalize_yaw_span_deg_ * M_PI / 180.0;
    const double yaw_step = relocalize_yaw_step_deg_ * M_PI / 180.0;
    const ros::WallTime started = ros::WallTime::now();
    double best_score = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f best = Eigen::Matrix4f::Identity();
    double near_cost = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f near_pose = Eigen::Matrix4f::Identity();
    double near_score = std::numeric_limits<double>::infinity();
    bool near_found = false;
    int attempts = 0;
    bool exhausted = false;
    std::vector<std::array<double, 3>> seeds;
    const int xy_count = std::max(1, static_cast<int>(
        std::floor(relocalize_radius_m_ / relocalize_xy_step_m_ + 1e-9)));
    for (int ix = -xy_count; ix <= xy_count; ++ix) {
      for (int iy = -xy_count; iy <= xy_count; ++iy) {
        for (double dyaw = -yaw_span; dyaw <= yaw_span + 1e-9; dyaw += yaw_step) {
          seeds.push_back({ix * relocalize_xy_step_m_, iy * relocalize_xy_step_m_, dyaw});
        }
      }
    }
    // Near-center seeds first: a bounded budget must spend its time on the
    // short-loss hypothesis, not on the search box corners.
    std::sort(seeds.begin(), seeds.end(), [](const std::array<double, 3>& a,
                                             const std::array<double, 3>& b) {
      const double da = a[0] * a[0] + a[1] * a[1] + 1e-6 * a[2] * a[2];
      const double db = b[0] * b[0] + b[1] * b[1] + 1e-6 * b[2] * b[2];
      return da < db;
    });
    Cloud out;
    for (const auto& offset : seeds) {
      if ((ros::WallTime::now() - started).toSec() > relocalize_budget_s_) {
        exhausted = true;
        break;
      }
      const Eigen::Matrix4f seed = makeT(center_x + offset[0], center_y + offset[1], 0.0,
                                         0.0, 0.0, center_yaw + offset[2]);
      ndt_.align(out, seed * t_base_lidar_);
      ++attempts;
      if (!ndt_.hasConverged()) continue;
      const double candidate_score = ndt_.getFitnessScore();
      if (candidate_score < best_score) {
        best_score = candidate_score;
        best = ndt_.getFinalTransformation() * t_lidar_base_;
      }
      if (candidate_score < relocalize_score_max_) {
        Eigen::Matrix4f candidate = ndt_.getFinalTransformation() * t_lidar_base_;
        const double displacement = std::hypot(candidate(0, 3) - center_x,
                                               candidate(1, 3) - center_y);
        const float candidate_yaw = std::atan2(candidate(1, 0), candidate(0, 0));
        const double yaw_difference = std::fabs(std::atan2(
            std::sin(candidate_yaw - center_yaw), std::cos(candidate_yaw - center_yaw)));
        if (displacement <= relocalize_max_displacement_m_
            && yaw_difference <= yaw_span + yaw_step) {
          // R1079: short-loss recovery trusts the nearest accepted hypothesis,
          // not the globally lowest fitness (the scene has lower-fitness basins).
          const double cost = displacement + 0.05 * yaw_difference;
          if (cost < near_cost) {
            near_cost = cost;
            near_score = candidate_score;
            near_pose = candidate;
            near_found = true;
          }
        }
      }
    }
    relocalize_attempts_ = attempts;
    relocalize_exhausted_ = exhausted;
    ROS_INFO("[v8_ndt] relocalize attempts=%d %s best=%.4f", attempts,
             exhausted ? "budget-exhausted" : "complete", best_score);
    relocalize_best_score_ = near_score;
    if (exhausted) {
      // An incomplete grid has not tested the hypotheses it promised; accepting
      // its incumbent would advertise a search that never finished.
      return false;
    }
    if (!near_found) return false;
    t_base_map_ = near_pose;
    return true;
  }

  void publishPoseOdometry(const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
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
    // No covariance calibration exists for this monitor; do not imply perfect certainty.
    for (int i = 0; i < 6; ++i) odom.pose.covariance[i * 6 + i] = 1e6;
    odom_pub_.publish(odom);
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (cloud->size() < 100) {
      publishHealth(false);
      return;
    }
    Cloud::Ptr ds(new Cloud);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setInputCloud(cloud);
    vg.filter(*ds);

    ndt_.setInputSource(ds);
    if (one_shot_relocalize_) {
      one_shot_relocalize_ = false;
      if (!boundedRelocalize()) {
        accept_align_ = false;
        relocalize_requested_ = false;
        std_msgs::Float32MultiArray dbg;
        dbg.data = {0.0f, -1.0f, ndt_.hasConverged() ? 1.0f : 0.0f};
        dbg_pub_.publish(dbg);
        publishHealth(false);
        return;
      }
      accept_align_ = true;
      history_.reset(t_base_map_);
      have_pose_ = true;
      history_.observe(t_base_map_, msg->header.stamp.toSec(), true);
      last_accept_score_ = relocalize_best_score_;
      std_msgs::Float32 relocalized_score;
      relocalized_score.data = static_cast<float>(relocalize_best_score_);
      score_pub_.publish(relocalized_score);
      publishPoseOdometry(msg->header.stamp);
      publishHealth(true);
      if (relocalize_requested_) {
        relocalize_requested_ = false;
        publishRelocalized(true);
      }
      std_msgs::Float32MultiArray dbg;
      dbg.data = {1.0f, (float)relocalize_best_score_, 1.0f};
      dbg_pub_.publish(dbg);
      return;  // Publish the search result directly; do not refine it into a worse basin.
    }
    if (!accept_align_) {
      // A failed bounded relocalization is fail-closed: no stale local guess may
      // silently become a fresh pose sample.
      publishHealth(false);
      return;
    }
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
        history_.reset(t_base_map_);
        ROS_INFO("[v8_ndt] global search best score=%.4f", best_score);
      }
    }
    Eigen::Matrix4f guess = history_.predict(use_prediction_) * t_base_lidar_;
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
        // R774：跳变不再按“连续 20 帧拒绝”放行——那会在对称走廊里把 180° 翻转
        // 放进来（v29_2 实测 max 5.55 m、yaw 179.5°）。改为双条件：
        // ①候选位姿自洽 ≥10 帧（相邻候选 <0.10 m / 2°）；②分数显著优于已跟踪位姿（<0.5×）。
        Eigen::Matrix4f rel2 = Eigen::Matrix4f::Identity();
        bool consistent = false;
        if (have_rej_pose_) {
          rel2 = rej_pose_.inverse() * t_new;
          consistent = rel2.block<3, 1>(0, 3).norm() < 0.10f
                       && fabs(atan2(rel2(1, 0), rel2(0, 0))) < 0.035f;
        }
        rej_consistent_ = consistent ? rej_consistent_ + 1 : 1;
        rej_pose_ = t_new;
        have_rej_pose_ = true;
        if (rej_consistent_ >= 10 && score < 0.5 * last_accept_score_) {
          rej_consistent_ = 0;
          have_rej_pose_ = false;
        } else {
          accept_new = false;
        }
      } else {
        rej_consistent_ = 0;
        have_rej_pose_ = false;
      }
    }
    const bool accepted = history_.observe(t_new, msg->header.stamp.toSec(), accept_new);
    std_msgs::Float32MultiArray dbg;
    dbg.data = {accepted ? 1.0f : 0.0f, (float)score, ndt_.hasConverged() ? 1.0f : 0.0f};
    dbg_pub_.publish(dbg);
    publishHealth(accepted);
    if (accepted) {
      t_base_map_ = t_lidar_map * t_lidar_base_;
      have_pose_ = true;
      last_accept_score_ = score;
      if (relocalize_requested_) {
        relocalize_requested_ = false;
        publishRelocalized(true);
      }
    }
    // A failed alignment is a failed observation, not a fresh sample of the old pose.
    if (!accepted) return;
    publishPoseOdometry(msg->header.stamp);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_, initialpose_sub_;
  ros::Publisher odom_pub_, score_pub_, dbg_pub_, health_pub_, relocalized_pub_;
  ros::ServiceServer reset_srv_, relocalize_srv_;
  pclomp::NormalDistributionsTransform<PointT, PointT> ndt_;
  Cloud::Ptr target_;
  Eigen::Matrix4f t_base_lidar_, t_lidar_base_, t_base_map_;
  bool have_pose_, global_search_, use_prediction_ = true;
  bool accept_align_ = true;
  bool relocalize_requested_ = false;
  bool one_shot_relocalize_ = false;
  bool relocalize_exhausted_ = false;
  int relocalize_attempts_ = 0;
  v8_ndt_odom::PoseHistory history_;
  Eigen::Matrix4f rej_pose_ = Eigen::Matrix4f::Identity();
  int rej_consistent_ = 0;
  bool have_rej_pose_ = false;
  double last_accept_score_ = 1e9;
  double resolution_, score_max_, ext_x_, ext_y_, ext_z_, ext_pitch_;
  int max_iterations_ = 30;
  double init_x_ = 0.0, init_y_ = 0.0, init_yaw_deg_ = 0.0;
  double relocalize_radius_m_, relocalize_xy_step_m_, relocalize_yaw_span_deg_;
  double relocalize_yaw_step_deg_, relocalize_budget_s_, relocalize_score_max_;
  double relocalize_max_displacement_m_;
  double relocalize_best_score_ = std::numeric_limits<double>::infinity();
  double step_size_ = 0.1;
  double gs_x_min_, gs_x_max_, gs_y_min_, gs_y_max_, gs_xy_step_, gs_yaw_step_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "v8_ndt_odom");
  NdtOdom node;
  ros::spin();
  return 0;
}
