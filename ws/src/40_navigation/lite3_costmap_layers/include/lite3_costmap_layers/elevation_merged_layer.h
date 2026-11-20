#ifndef LITE3_COSTMAP_LAYERS_ELEVATION_MERGED_LAYER_H_
#define LITE3_COSTMAP_LAYERS_ELEVATION_MERGED_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace lite3_costmap_layers
{
/**
 * @brief ElevationMergedLayer —— 高程 → 合并代价层（B 线，裁11／裁12 落地件）
 *
 * 定位（R441—R453 预检线已验证的路径）：
 *   - 角色与 sob_layer 相同＝costmap_2d::Layer，进 global/local_costmap 的 plugins 列表；
 *   - ★不另开第二层障碍层、不塞第二个 layer★（R59／X2 在册结论），由本层直接产出合并代价；
 *   - 合并语义显式钉 use_maximum: true（R445：false 时第二枚层可把底层致命格 100→0；
 *     R446：本镜像这条链路上"漏写"与"写 false"同形）。
 *
 * 输入：grid_map_msgs/GridMap（默认话题 /elevation_mapping/elevation_map，层名默认 elevation）。
 * 判据：格子高程 − ground_z > height_threshold ⇒ 该格写 obstacle_cost。
 *
 * 边界（本版刻意不做的事）：
 *   - 不做下采样、不做多帧融合、不发布可视化话题；
 *   - 不读任何门值文件、不改 nav 侧在册配置；
 *   - 坐标系：与 costmap 全局帧不同时用 tf_ 做平面变换（只取 yaw＋平移，不取 roll/pitch——
 *     代价图是 2D 的，roll/pitch 不在本层语义内）。
 */
class ElevationMergedLayer : public costmap_2d::Layer
{
public:
  ElevationMergedLayer() = default;
  ~ElevationMergedLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                    double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void activate() override;
  void deactivate() override;

private:
  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
  bool inputIsFresh(const ros::Time& stamp) const;
  bool lookupPlanarTransform(const std::string& source_frame, double& yaw, double& tx, double& ty);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  // 订阅门与基类更新门必须一致；缺少有效输入不能靠禁用层绕过。
  bool enabled_param_ = true;

  std::mutex data_mutex_;
  grid_map::GridMap map_;
  bool has_data_ = false;
  unsigned long callback_count_ = 0;   // R572：回调计数（判两侧同名层是否都在收）
  // ★R572 常设★：写入计数（判本层真的往主代价图写过东西，而不只是收到了 GridMap）。
  // 这四项与 callback_count_ 一起构成「本层活体四联」：收包数／写窗数／写格数／拒写次数。
  unsigned long write_windows_ = 0;
  unsigned long write_cells_ = 0;
  ros::Time last_map_stamp_;  // 无效输入不清除时间戳，防止旧帧重新生效。

  // 参数（onInitialize 里从 ~/<name> 读）
  std::string input_topic_ = "/elevation_mapping/elevation_map";
  std::string layer_name_ = "elevation";
  double input_timeout_s_ = 0.0;  // 0 表示未配置，不替部署选择 TTL。
  double height_threshold_ = 0.15;  // 我方自定值（X5 同族：step>0.15m），无在册实测锚
  double ground_z_ = 0.0;
  unsigned char obstacle_cost_ = costmap_2d::LETHAL_OBSTACLE;

  // use_maximum 三道状态：默认 true（漏写）／显式 true／显式 false（⇒ 拒绝写入）
  bool use_maximum_ = true;
  bool use_maximum_explicit_ = false;
  bool refuse_write_ = false;
  int refuse_log_count_ = 0;

  // 由 updateBounds 填、updateCosts 用（世界系坐标对，压平存放）
  std::vector<double> marked_xy_;
  bool frame_warned_ = false;
  bool layer_warned_ = false;
};

}  // namespace lite3_costmap_layers

#endif  // LITE3_COSTMAP_LAYERS_ELEVATION_MERGED_LAYER_H_
