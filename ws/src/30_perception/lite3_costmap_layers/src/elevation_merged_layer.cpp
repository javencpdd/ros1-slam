#include "lite3_costmap_layers/elevation_merged_layer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pluginlib/class_list_macros.hpp>

// 注册为 costmap_2d::Layer 插件；类型串＝C++ 类型（与在册 sob_layer::SobLayer 同约定）。
PLUGINLIB_EXPORT_CLASS(lite3_costmap_layers::ElevationMergedLayer, costmap_2d::Layer)

namespace lite3_costmap_layers
{
namespace
{
bool validMessageGeometry(const grid_map_msgs::GridMap& msg)
{
  const auto& info = msg.info;
  if (!std::isfinite(info.resolution) || info.resolution <= 0.0 ||
      !std::isfinite(info.length_x) || info.length_x <= 0.0 ||
      !std::isfinite(info.length_y) || info.length_y <= 0.0 ||
      !std::isfinite(info.pose.position.x) || !std::isfinite(info.pose.position.y))
    return false;

  // 转换器会直接分配矩阵并读取缓冲，不能等转换后再验证形状。
  const double rows = std::round(info.length_x / info.resolution);
  const double cols = std::round(info.length_y / info.resolution);
  if (!std::isfinite(rows) || !std::isfinite(cols) || rows <= 0.0 || cols <= 0.0 ||
      rows * cols > std::numeric_limits<int>::max() || msg.layers.size() != msg.data.size() ||
      msg.outer_start_index >= rows || msg.inner_start_index >= cols)
    return false;

  for (const auto& data : msg.data)
  {
    const auto& dims = data.layout.dim;
    if (dims.size() != 2 || dims[0].size != cols || dims[1].size != rows ||
        data.layout.data_offset != 0 || data.data.size() != static_cast<size_t>(rows * cols))
      return false;
  }
  return true;
}
}  // namespace

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
  pnh.param("enabled", enabled_param_, true);
  enabled_ = enabled_param_;
  pnh.param("input_timeout_s", input_timeout_s_, 0.0);
  if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0)
  {
    ROS_ERROR("[%s] input_timeout_s 必须显式配置为有限正数；本层保持 noncurrent", name_.c_str());
  }
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

  reset();

  if (enabled_param_)
  {
    sub_ = nh_.subscribe(input_topic_, 1, &ElevationMergedLayer::gridMapCallback, this);
    // 订阅是否真的建立，是「插件装载成功但一次 Received 都没有」这枚静默失效的唯一分界。
    // ★R572 起本行常设★：本批就是靠「参数为 true 仍不起」才发现基类 enabled_ 恒 false。
    ROS_INFO("[%s] 订阅已建立：topic=%s（解析后=%s）enabled_param=%d", name_.c_str(),
             input_topic_.c_str(), sub_.getTopic().c_str(), static_cast<int>(enabled_param_));
  }
  else
  {
    ROS_ERROR("[%s] ★enabled=false ⇒ 未建立订阅★（本层将永远不写代价，且不会有任何 Received 行）",
              name_.c_str());
  }
}

void ElevationMergedLayer::activate()
{
  // ★R572★：activate 也必须守同一条门（enabled_param_），否则 onInitialize 判 false 之后
  // 一次 activate 就把订阅又接上了 —— 两种路径口径不一，是同类静默漂移的温床。
  if (enabled_param_ && !sub_)
  {
    sub_ = nh_.subscribe(input_topic_, 1, &ElevationMergedLayer::gridMapCallback, this);
    ROS_INFO("[%s] activate 后补建订阅：%s", name_.c_str(), sub_.getTopic().c_str());
  }
}

void ElevationMergedLayer::deactivate()
{
  sub_.shutdown();
  reset();
}

void ElevationMergedLayer::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto reject = [this](const char* reason) {
    has_data_ = false;
    ROS_ERROR_THROTTLE(1.0, "[%s] GridMap 输入无效：%s", name_.c_str(), reason);
  };
  grid_map::GridMap candidate;
  try
  {
    if (!inputIsFresh(msg->info.header.stamp) || msg->info.header.frame_id.empty())
    {
      reject("缺少有效 TTL、帧或采集时间戳，或输入已过期/来自未来");
      return;
    }
    if (!validMessageGeometry(*msg))
    {
      reject("几何或层缓冲尺寸无效");
      return;
    }
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, candidate))
    {
      reject("转换器返回 false");
      return;
    }
    const auto size = candidate.getSize();
    if (size(0) <= 0 || size(1) <= 0 || !std::isfinite(candidate.getResolution()) ||
        candidate.getResolution() <= 0.0 || !std::isfinite(candidate.getLength()(0)) ||
        !std::isfinite(candidate.getLength()(1)) || candidate.getLength()(0) <= 0.0 ||
        candidate.getLength()(1) <= 0.0 || !candidate.exists(layer_name_))
    {
      reject("缺少所需层或转换后几何无效");
      return;
    }
    const grid_map::Matrix& data = candidate[layer_name_];
    if (data.rows() != size(0) || data.cols() != size(1) ||
        !std::any_of(data.data(), data.data() + data.size(), [](float h) { return std::isfinite(h); }))
    {
      reject("所需层尺寸无效或没有有限观测");
      return;
    }
    // 重发不续期，乱序帧不能复活已失效的旧输入。
    if (msg->info.header.stamp <= last_map_stamp_)
      return;
    map_ = std::move(candidate);
  }
  catch (const std::exception& e)
  {
    reject(e.what());
    return;
  }
  last_map_stamp_ = msg->info.header.stamp;
  has_data_ = true;
  ++callback_count_;
  // ★主证行（R572 改法）★：原先是 ROS_INFO_THROTTLE(1.0, "Received a grid map…")。
  // 那写法在本层有一个坑：ROS_INFO_THROTTLE 的节流状态按**调用点**记，是整进程共享的静态量，
  // 而 global_costmap 与 local_costmap 两支的 ElevationMergedLayer 跑在**同一个 mbf 进程**里
  // ⇒ local 每秒打印一次就把 global 的配额吞掉，global 侧永远不出现这一行。
  // 本批实测：两侧回调计数 78:78 完全对称，而 Received 行 global 0 / local 8。
  // 处置＝主证行去掉节流、改成回调计数（每拍一行，名册直接可数），节流只留给它下面的采样行。
  ROS_INFO("[%s] Received a grid map #%lu（%ux%u、frame=%s、stamp=%.3f）", name_.c_str(),
           static_cast<unsigned long>(callback_count_), map_.getSize()(0), map_.getSize()(1),
           map_.getFrameId().c_str(), last_map_stamp_.toSec());
}

void ElevationMergedLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                        double* min_x, double* min_y, double* max_x, double* max_y)
{
  grid_map::GridMap map;
  ros::Time stamp;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_ = false;
    marked_xy_.clear();
    if (!enabled_param_ || refuse_write_ || !has_data_ || !inputIsFresh(last_map_stamp_))
      return;
    map = map_;
    stamp = last_map_stamp_;
  }

  if (!map.exists(layer_name_) || map.getFrameId().empty())
    return;

  const std::string global_frame = (layered_costmap_ != nullptr) ? layered_costmap_->getGlobalFrameID() : "";
  if (global_frame.empty())
    return;
  const bool needs_transform = map.getFrameId() != global_frame;
  double yaw = 0.0, tx = 0.0, ty = 0.0;
  if (needs_transform)
  {
    if (!frame_warned_)
    {
      frame_warned_ = true;
      ROS_WARN("[%s] GridMap 帧 %s 与全局帧 %s 不同 ⇒ 本层按 tf_ 做平面变换（只用 yaw＋平移）", name_.c_str(),
               map.getFrameId().c_str(), global_frame.c_str());
    }
    // 平地无障碍也必须具备可用变换；一窗只用同一个 TF 快照。
    if (!lookupPlanarTransform(map.getFrameId(), yaw, tx, ty))
      return;
  }
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  std::vector<double> marked_xy;
  bool has_observation = false;
  const grid_map::Matrix& data = map[layer_name_];
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    const grid_map::Index index(*it);
    const float h = data(index(0), index(1));
    if (!std::isfinite(h))
    {
      continue;
    }
    has_observation = true;
    if (static_cast<double>(h) <= ground_z_ + height_threshold_)
    {
      continue;
    }
    grid_map::Position position;
    if (!map.getPosition(index, position))
      return;
    double wx = position.x();
    double wy = position.y();
    if (needs_transform)
    {
      const double nx = cy * wx - sy * wy + tx;
      const double ny = sy * wx + cy * wy + ty;
      wx = nx;
      wy = ny;
    }
    if (!std::isfinite(wx) || !std::isfinite(wy))
      return;
    marked_xy.push_back(wx);
    marked_xy.push_back(wy);
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  // 回调或 reset 可能已淘汰本窗快照，处理耗时也不能续期。
  if (!has_observation || !has_data_ || last_map_stamp_ != stamp || !inputIsFresh(stamp))
    return;
  marked_xy_.swap(marked_xy);
  for (size_t k = 0; k + 1 < marked_xy_.size(); k += 2)
  {
    *min_x = std::min(*min_x, marked_xy_[k]);
    *max_x = std::max(*max_x, marked_xy_[k]);
    *min_y = std::min(*min_y, marked_xy_[k + 1]);
    *max_y = std::max(*max_y, marked_xy_[k + 1]);
  }
  current_ = true;
}

void ElevationMergedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                       int max_j)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!enabled_param_ || refuse_write_)
  {
    // ★R572 负向控制带出来的真缺陷（本批第三枚）★
    // 原形是 (refuse_log_count_++ % 100 == 0)：可 updateCosts 从不被跳过时这个计数器恒 0，
    // 而拒绝写入恰恰就是「每次都跳过」⇒ 第一次进这里 ++ 后变 1、1 % 100 != 0 ⇒ 一行都不打。
    // 实测（R572 neg6，use_maximum=false 窗）：refuse_write=1 打印 2 次，本行打印 0 次。
    // 即：安全侧的一条 ROS_ERROR 在生效时反而是静默的 —— 与 enabled_ 那枚同型（静默失效）。
    // 改成「前 3 次逐次打、之后每 100 次打一次」，保证它一旦生效必留痕。
    if (refuse_write_)
    {
      ++refuse_log_count_;
      if (refuse_log_count_ <= 3 || refuse_log_count_ % 100 == 0)
      {
        ROS_ERROR("[%s] use_maximum=false ⇒ 本层不写任何代价（第 %d 次 updateCosts 被跳过）", name_.c_str(),
                  refuse_log_count_);
      }
    }
    return;
  }
  if (!current_ || !has_data_ || !inputIsFresh(last_map_stamp_))
  {
    current_ = false;
    marked_xy_.clear();
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
  // ★R572 第三处改法：写格数从 ROS_DEBUG 提升为常设计数★
  // 原先是 ROS_DEBUG ⇒ 默认级别下这行永远不可见，于是「本层到底写没写代价」在日志上无从判定。
  // 本批正是因此绕了一大圈：Received 双侧齐、在位行齐、却查不出代价是否落盘，
  // 最后只能靠 check_pose_cost 的差别反推 —— 而那条判据在 static_layer 已铺满台阶时不成立。
  // 改成累计写入格数与窗口计数，一行可数，才谈得上「活体代价签名」。
  write_windows_ += 1;
  write_cells_ += written;
  ROS_INFO("[%s] 本窗写入 %u 格（累计 %lu 窗 / %lu 格；use_maximum=%d、res=%.3f）", name_.c_str(), written,
           static_cast<unsigned long>(write_windows_), static_cast<unsigned long>(write_cells_), use_maximum_, res);
}

void ElevationMergedLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  marked_xy_.clear();
  map_ = grid_map::GridMap();
  last_map_stamp_ = ros::Time();
  has_data_ = false;
  current_ = false;
}

bool ElevationMergedLayer::inputIsFresh(const ros::Time& stamp) const
{
  if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0 || stamp.isZero())
    return false;
  const ros::Time now = ros::Time::now();
  if (now < stamp)
    return false;
  const double age = static_cast<double>(now.toNSec() - stamp.toNSec()) / 1e9;
  return age <= input_timeout_s_;
}

bool ElevationMergedLayer::lookupPlanarTransform(const std::string& source_frame, double& yaw, double& tx, double& ty)
{
  if (tf_ == nullptr || layered_costmap_ == nullptr)
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
  geometry_msgs::Quaternion q = tf.transform.rotation;
  const auto& translation = tf.transform.translation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(translation.x) || !std::isfinite(translation.y) || !std::isfinite(translation.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      !std::isfinite(norm) || norm <= 0.0)
  {
    ROS_WARN_THROTTLE(1.0, "[%s] TF 平移或四元数无效", name_.c_str());
    return false;
  }
  // 归一化而非引入部署相关的姿态门槛。
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
  tx = translation.x;
  ty = translation.y;
  return std::isfinite(yaw);
}

}  // namespace lite3_costmap_layers
