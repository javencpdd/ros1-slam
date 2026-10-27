// 静态 score 探针：给定地图/一帧点云/初始位姿，报告 NDT fitness。
// fixed 模式（--fixed）：最大迭代 1、步长 0 ⇒ 不优化，直接返回该位姿下的 score，
// 用于「静止真位姿分布」与「错位正控」。
// 用法：ndt_score_probe <map.pcd> <scan.pcd> <x> <y> <yaw_deg> [res] [align|fixed]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

using PointT = pcl::PointXYZI;

static Eigen::Matrix4f makeT(double x, double y, double yaw) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = cos(yaw); T(0, 1) = -sin(yaw);
  T(1, 0) = sin(yaw); T(1, 1) = cos(yaw);
  T(0, 3) = x; T(1, 3) = y;
  return T;
}

int main(int argc, char** argv) {
  if (argc < 6) {
    fprintf(stderr,
            "usage: ndt_score_probe <map.pcd> <scan.pcd> <x> <y> <yaw_deg> "
            "[resolution] [align|fixed]\n");
    return 9;
  }
  const char* map_path = argv[1];
  const char* scan_path = argv[2];
  const double x = atof(argv[3]);
  const double y = atof(argv[4]);
  const double yaw = atof(argv[5]) * M_PI / 180.0;
  const double res = argc > 6 ? atof(argv[6]) : 1.0;
  const bool fixed = argc > 7 && strcmp(argv[7], "fixed") == 0;
  const bool resid = argc > 7 && strcmp(argv[7], "resid") == 0;

  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr scan(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(map_path, *map) < 0 || map->empty()) {
    fprintf(stderr, "cannot load map %s\n", map_path);
    return 3;
  }
  if (pcl::io::loadPCDFile<PointT>(scan_path, *scan) < 0 || scan->empty()) {
    fprintf(stderr, "cannot load scan %s\n", scan_path);
    return 3;
  }

  pcl::PointCloud<PointT>::Ptr map_ds(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  vg.setInputCloud(map);
  vg.filter(*map_ds);
  pcl::PointCloud<PointT>::Ptr scan_ds(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg2;
  vg2.setLeafSize(0.1f, 0.1f, 0.1f);
  vg2.setInputCloud(scan);
  vg2.filter(*scan_ds);

  if (resid) {
    // 点到点残差：把 scan 按给定位姿变换后，取每点到地图最近邻距离的均值/平方均值。
    pcl::PointCloud<PointT>::Ptr map_pts(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg3;
    vg3.setLeafSize(0.05f, 0.05f, 0.05f);
    vg3.setInputCloud(map);
    vg3.filter(*map_pts);
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(map_pts);
    Eigen::Matrix4f T = makeT(x, y, yaw);
    double sum = 0.0, sum2 = 0.0;
    int n = 0;
    std::vector<int> idx(1);
    std::vector<float> d2(1);
    for (const auto& p : scan_ds->points) {
      Eigen::Vector4f q = T * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
      PointT query;
      query.x = q(0); query.y = q(1); query.z = q(2);
      if (kdtree.nearestKSearch(query, 1, idx, d2) > 0) {
        const double d = std::sqrt(std::max(0.0f, d2[0]));
        sum += d;
        sum2 += d * d;
        n++;
      }
    }
    printf("{\"score\": %.6f, \"rms\": %.6f, \"converged\": true, \"iterations\": 0, "
           "\"x\": %.4f, \"y\": %.4f, \"yaw_deg\": %.4f, \"mode\": \"resid\", \"res\": %.2f}\n",
           n ? sum / n : -1.0, n ? std::sqrt(sum2 / n) : -1.0, x, y, yaw * 180.0 / M_PI, res);
    return 0;
  }

  pclomp::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setResolution(res);
  ndt.setInputTarget(map_ds);
  ndt.setMaximumIterations(fixed ? 1 : 30);
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(fixed ? 0.0 : 0.1);
  ndt.setNumThreads(4);

  pcl::PointCloud<PointT> out;
  ndt.setInputSource(scan_ds);
  ndt.align(out, makeT(x, y, yaw));
  Eigen::Matrix4f T = ndt.getFinalTransformation();
  const double sc = ndt.getFitnessScore();
  printf("{\"score\": %.6f, \"converged\": %s, \"iterations\": %d, "
         "\"x\": %.4f, \"y\": %.4f, \"yaw_deg\": %.4f, \"mode\": \"%s\", \"res\": %.2f}\n",
         sc, ndt.hasConverged() ? "true" : "false", (int)ndt.getFinalNumIteration(),
         T(0, 3), T(1, 3), atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI,
         fixed ? "fixed" : "align", res);
  return 0;
}
