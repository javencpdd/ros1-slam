#include "lite3_costmap_layers/elevation_merged_layer.h"

#include <algorithm>
#include <cmath>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pluginlib/class_list_macros.hpp>

// 注册为 costmap_2d::Layer 插件；类型串＝C++ 类型（与在册 sob_layer::SobLayer 同约定）。
PLUGINLIB_EXPORT_CLASS(lite3_costmap_layers::ElevationMergedLayer, costmap_2d::Layer)

namespace lite3_costmap_layers
{

void ElevationMergedLayer::onInitialize()
{
  ros::NodeHandle pnh("~/" + name_);
  nh_ = pnh;

  // ---- use_maximum：本层最重要的一枚开关（R445／R446）----
  // 本镜像这条链路上"漏写＝覆盖、写 false＝安全侧反例"（R445：false 时第二枚层把底层
  // 致命格 100→0；R446：漏写两个组合键 ⇒ 非零 9→0，与显式 false 同形）。
  // 处置分三档，绝不静默：
  //   ① 没给        ⇒ 取 true，并 WARN 一次（每次 onInitialize 一次）；
  //   ② 显式 true   ⇒ 正常合并（取大，永不抹低）；
  //   ③ 显式 false  ⇒ ERROR，且 refuse_write_＝true（本层不写任何代价，宁可漏报不覆盖）。
  if (!pnh.getParam("use_maximum", use_maximum_))
  {
    use_maximum_ = true;
    ROS_WARN("[%s] 参数 use_maximum 未给 ⇒ 取默认 true（在册 R445/R446：漏写与写 false 同形）", name_.c_str());
  }
  else
  {
    use_maximum_explicit_ = true;
    if (!use_maximum_)
    {
      refuse_write_ = true;
      ROS_ERROR("[%s] ★use_maximum=false 被显式写入 ⇒ 本层拒绝写入任何代价★（R445：false 时可抹掉底层致命格）",
                name_.c_str());
    }
  }

  pnh.param("input_topic", input_topic_, std::string("/elevation_mapping/elevation_map"));
  pnh.param("layer_name", layer_name_, std::string("elevation"));
  pnh.param("height_threshold", height_threshold_, 0.15);
  pnh.param("ground_z", ground_z_, 0.0);
  int cost = static_cast<int>(costmap_2d::LETHAL_OBSTACLE);
  pnh.param("obstacle_cost", cost, static_cast<int>(costmap_2d::LETHAL_OBSTACLE));
  obstacle_cost_ = static_cast<unsigned char>(cost);

  if (layered_costmap_ != nullptr)
  {
    ROS_INFO("[%s] 在位：input=%s layer=%s global_frame=%s use_maximum=%d(explicit=%d refuse_write=%d) "
             "height_threshold=%.3f ground_z=%.3f obstacle_cost=%d",
             name_.c_str(), input_topic_.c_str(), layer_name_.c_str(),
             layered_costmap_->getGlobalFrameID().c_str(), use_maximum_, use_maximum_explicit_, refuse_write_,
             height_threshold_, ground_z_, static_cast<int>(obstacle_cost_));
  }
  else
  {
    ROS_WARN("[%s] 在位但 layered_costmap_ 为空（不读全局帧）", name_.c_str());
  }

  marked_xy_.clear();
  has_data_ = false;

  if (enabled_)
  {
    sub_ = nh_.subscribe(input_topic_, 1, &ElevationMergedLayer::gridMapCallback, this);
  }
}

void ElevationMergedLayer::activate()
{
  if (!sub_)
  {
    sub_ = nh_.subscribe(input_topic_, 1, &ElevationMergedLayer::gridMapCallback, this);
  }
}

void ElevationMergedLayer::deactivate()
{
  sub_.shutdown();
}

void ElevationMergedLayer::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  try
  {
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s] GridMap 转换失败：%s", name_.c_str(), e.what());
    return;
  }
  if (!map_.exists(layer_name_))
  {
    ROS_ERROR_THROTTLE(1.0, "[%s] 收到的 GridMap 里没有层 %s（现有层见 info 日志）", name_.c_str(), layer_name_.c_str());
    return;
  }
  last_map_stamp_ = msg->info.header.stamp;
  has_data_ = true;
  // ★主证行★（沿在册 R444 的取证式：节点日志里必须有"收到了"这一行，名册报 None 不作数）
  ROS_INFO_THROTTLE(1.0, "[%s] Received a grid map（%ux%u、frame=%s、stamp=%.3f）", name_.c_str(),
                    map_.getSize()(0), map_.getSize()(1), map_.getFrameId().c_str(), last_map_stamp_.toSec());
}

void ElevationMergedLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                        double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !has_data_)
  {
    marked_xy_.clear();
    return;
  }

  grid_map::GridMap map;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map = map_;  // GridMap 拷贝（Buffer 是 Eigen 矩阵，深拷贝）
  }

  if (!map.exists(layer_name_))
  {
    marked_xy_.clear();
    return;
  }

  const std::string global_frame = (layered_costmap_ != nullptr) ? layered_costmap_->getGlobalFrameID() : "";
  if (!global_frame.empty() && map.getFrameId() != global_frame && !frame_warned_)
  {
    frame_warned_ = true;
    ROS_WARN("[%s] GridMap 帧 %s 与全局帧 %s 不同 ⇒ 本层按 tf_ 做平面变换（只用 yaw＋平移）", name_.c_str(),
             map.getFrameId().c_str(), global_frame.c_str());
  }

  marked_xy_.clear();
  const grid_map::Matrix& data = map[layer_name_];
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    const grid_map::Index index(*it);
    const float h = data(index(0), index(1));
    if (!std::isfinite(h))
    {
      continue;
    }
    if (static_cast<double>(h) <= ground_z_ + height_threshold_)
    {
      continue;
    }
    grid_map::Position position;
    if (!map.getPosition(index, position))
    {
      continue;
    }
    double wx = position.x();
    double wy = position.y();
    if (!global_frame.empty() && map.getFrameId() != global_frame)
    {
      if (!transformToGlobal(map.getFrameId(), wx, wy))
      {
        continue;
      }
    }
    if (wx < *min_x)
      *min_x = wx;
    if (wx > *max_x)
      *max_x = wx;
    if (wy < *min_y)
      *min_y = wy;
    if (wy > *max_y)
      *max_y = wy;
    marked_xy_.push_back(wx);
    marked_xy_.push_back(wy);
  }
}

void ElevationMergedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                       int max_j)
{
  if (!enabled_ || refuse_write_)
  {
    if (refuse_write_ && (refuse_log_count_++ % 100 == 0))
    {
      ROS_ERROR("[%s] use_maximum=false ⇒ 本层不写任何代价（第 %d 次 updateCosts 被跳过）", name_.c_str(),
                refuse_log_count_);
    }
    return;
  }
  if (marked_xy_.size() < 2)
  {
    return;
  }

  const double res = master_grid.getResolution();
  unsigned int written = 0;
  for (size_t k = 0; k + 1 < marked_xy_.size(); k += 2)
  {
    const double wx = marked_xy_[k];
    const double wy = marked_xy_[k + 1];
    unsigned int mx, my;
    if (!master_grid.worldToMap(wx, wy, mx, my))
    {
      continue;
    }
    if (static_cast<int>(mx) < min_i || static_cast<int>(mx) > max_i || static_cast<int>(my) < min_j ||
        static_cast<int>(my) > max_j)
    {
      continue;
    }
    const unsigned char old_cost = master_grid.getCost(mx, my);
    unsigned char new_cost = obstacle_cost_;
    if (use_maximum_)
    {
      new_cost = std::max(old_cost, new_cost);  // 取大 ⇒ 永不把已有的高代价抹低
    }
    master_grid.setCost(mx, my, new_cost);
    ++written;
  }
  ROS_DEBUG("[%s] 本窗写入 %u 格（use_maximum=%d、costmap 分辨率 %.3f）", name_.c_str(), written, use_maximum_, res);
}

void ElevationMergedLayer::reset()
{
  marked_xy_.clear();
  has_data_ = false;
}

bool ElevationMergedLayer::transformToGlobal(const std::string& source_frame, double& x, double& y)
{
  if (tf_ == nullptr)
  {
    return false;
  }
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), source_frame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "[%s] tf 查询失败 %s -> %s：%s", name_.c_str(), source_frame.c_str(),
                      layered_costmap_->getGlobalFrameID().c_str(), ex.what());
    return false;
  }
  const geometry_msgs::Quaternion& q = tf.transform.rotation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double tx = tf.transform.translation.x;
  const double ty = tf.transform.translation.y;
  const double nx = cy * x - sy * y + tx;
  const double ny = sy * x + cy * y + ty;
  x = nx;
  y = ny;
  return true;
}

}  // namespace lite3_costmap_layers
