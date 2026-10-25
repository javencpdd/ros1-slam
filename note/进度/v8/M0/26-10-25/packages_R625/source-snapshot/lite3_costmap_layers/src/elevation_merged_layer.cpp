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
  // ★R572（批224）：本层自有启用开关，缺省 true★。
  // 不再复用基类 enabled_（它构造即 false、且 costmap_2d 不读 enabled 参数，见头文件长注释）。
  pnh.param("enabled", enabled_param_, true);
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

  // ★R572 第二枚静默失效的修法（比订阅门更隐蔽，且是本批真正卡住代价的那一道）★
  // 订阅建立了、GridMap 也在收（两侧 74/74 对称），代价却一格没写 —— 因为
  // LayeredCostmap::updateMap() @0x4b9c0 是 `cmpb $0x0,0x11(%rdi); je 跳过` 逐层筛的，
  // 而 Layer::Layer() 把 enabled_（0x11）置 0 之后，Layer::initialize() 全文都不写它
  // ⇒ 任何不自带启用的层都被框架静默排除出更新回路，updateBounds/updateCosts 一次不进。
  // 必须在 onInitialize 末尾显式置 true，否则上面建立的订阅只是「收得到、用不上」。
  enabled_ = true;
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
  if (!enabled_param_ || !has_data_)
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
