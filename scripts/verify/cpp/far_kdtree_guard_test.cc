// FAR 空树防护的 C++ 反控（2026-11-26 裁决①）。
//
// 被验的对象：`MapHandler::NearestHeightOfRadius` / `NearestHeightOfPoint` 在
// `kdtree_terrain_clould_` 为空指针、空云、有效云、清空后重建 四种状态下的行为。
//
// 为什么必须有它：SIGSEGV 是"地形还没到就调用 radiusSearch"造成的，
// 只有真跑 C++ 才能证明"没地形时不崩，而且不会吐出未经验证的高度"。
// Python 侧的桩测试证明不了这一点。
//
// 判据（4 组）：
//   1) 未初始化地形（nullptr）      → 返回 p.z，is_matched=false，不崩
//   2) 空地形（setInputCloud 但 0 点）→ 返回 p.z，is_matched=false，不崩（这正是原崩溃现场）
//   3) 有效地形（3 点）             → 返回半径内 intensity 均值，is_matched=true
//   4) 清空后重建索引               → 先回到 p.z，重建后再给出均值
#include <cstdio>
#include <cmath>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// 访问被测类的静态成员（它只含静态函数与静态指针，这里用 `#define private public`
// 是为了**不修改上游头文件**，测完立即还原）
#define private public
#define protected public
#include "far_planner/map_handler.h"
#undef protected
#undef private

// 静态成员的定义本来在 map_handler.cpp 里；本测试只链头文件里的 inline 实现，
// 不拉整个 map_handler.cpp（它会带进 ROS/可视化一堆依赖），所以在这里补上定义。
PointKdTreePtr MapHandler::kdtree_terrain_clould_;

namespace {

int g_failed = 0;

void check(bool ok, const char* what) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++g_failed;
}

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using Tree = pcl::KdTreeFLANN<pcl::PointXYZI>;

// 半径为 1.0 的查询点，intensity 承载"地形高度"
struct P3 { float x, y, z; };

float callRadius(float x, float y, float z, bool& matched) {
  float minH = 0.f, maxH = 0.f;
  return MapHandler::NearestHeightOfRadius(P3{x, y, z}, 1.0f, minH, maxH, matched);
}

}  // namespace

int main() {
  // ---- 1) 未初始化：nullptr ----
  MapHandler::kdtree_terrain_clould_.reset();
  {
    bool matched = true;
    const float h = callRadius(0.f, 0.f, 7.5f, matched);
    check(h == 7.5f && !matched, "未初始化地形：返回 p.z 且 is_matched=false");
  }

  // ---- 2) 空地形（原崩溃现场：tree 存在但 0 点）----
  {
    Cloud::Ptr empty(new Cloud());
    Tree::Ptr tree(new Tree());
    tree->setInputCloud(empty);
    MapHandler::kdtree_terrain_clould_ = tree;
    bool matched = true;
    const float h = callRadius(0.f, 0.f, 3.25f, matched);
    check(h == 3.25f && !matched, "空地形：返回 p.z 且 is_matched=false（不再 SIGSEGV）");
  }

  // ---- 3) 有效地形：三点 intensity=1/2/3，查询半径 1.0 ----
  {
    Cloud::Ptr cloud(new Cloud());
    const float heights[3] = {1.0f, 2.0f, 3.0f};
    for (int i = 0; i < 3; ++i) {
      pcl::PointXYZI pt;
      pt.x = 0.1f * i; pt.y = 0.f; pt.z = 0.f;
      pt.intensity = heights[i];
      cloud->push_back(pt);
    }
    Tree::Ptr tree(new Tree());
    tree->setInputCloud(cloud);
    MapHandler::kdtree_terrain_clould_ = tree;
    bool matched = false;
    const float h = callRadius(0.f, 0.f, 9.0f, matched);
    check(matched && std::fabs(h - 2.0f) < 1e-5f, "有效地形：返回半径内均值 2.0 且 is_matched=true");
  }

  // ---- 4) 清空后重建 ----
  {
    Cloud::Ptr cloud(new Cloud());
    Tree::Ptr tree(new Tree());
    tree->setInputCloud(cloud);
    MapHandler::kdtree_terrain_clould_ = tree;
    bool matched = true;
    const float h1 = callRadius(0.f, 0.f, -1.0f, matched);
    check(h1 == -1.0f && !matched, "清空索引：回到 p.z 且 is_matched=false");

    for (int i = 0; i < 2; ++i) {
      pcl::PointXYZI pt;
      pt.x = 0.f; pt.y = 0.f; pt.z = 0.f;
      pt.intensity = 4.0f;
      cloud->push_back(pt);
    }
    tree->setInputCloud(cloud);
    MapHandler::kdtree_terrain_clould_ = tree;
    matched = false;
    const float h2 = callRadius(0.f, 0.f, 0.0f, matched);
    check(matched && std::fabs(h2 - 4.0f) < 1e-5f, "重建索引：再次给出 4.0 且 is_matched=true");
  }

  // ---- 5) 最近邻同款防护 ----
  {
    MapHandler::kdtree_terrain_clould_.reset();
    float d2 = -1.f;
    const float h = MapHandler::NearestHeightOfPoint(P3{0.f, 0.f, 5.5f}, d2);
    check(h == 5.5f, "最近邻：无地形时返回 p.z");
  }

  std::printf("RESULT %s failed=%d\n", g_failed == 0 ? "PASS" : "FAIL", g_failed);
  return g_failed == 0 ? 0 : 1;
}
