#include "pointcloud_preprocess.h"

#include <glog/logging.h>
#include <execution>

namespace faster_lio {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    AviaHandler(msg);
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::C16:
            C16Handler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].curvature =
                    msg->points[i].offset_time /
                    float(1000000);  // use curvature as time of each laser points, curvature unit: ms

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                        (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                             cloud_full_[i].z * cloud_full_[i].z >
                         (blind_ * blind_))) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) continue;

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = pl_orig.points[i].t / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }
}

void PointCloudPreprocess::C16Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<leishen_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (C16SelfMaskHit(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z, range, blind_)) continue;  // C13=乙(2026-10-11 主理人裁「原地换形状」)：本行仍是**唯一生效点**，四道显式判据(非有限/精确零/机体盒/blind 兜底球)见本文件尾定义

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;

        cloud_out_.points.push_back(added_pt);
    }
}

}  // namespace faster_lio


// ============================================================================================
// C13＝乙（2026-10-11 主理人裁定"原地换形状"）——C16Handler 的自遮挡/占位点判据
// --------------------------------------------------------------------------------------------
// 为什么在这：球形 `blind` 把 0.56–1.0m 一层**真实障碍**连自回波一起剔掉（贴地障碍、楼梯第一级立面、桌沿、人的脚部
//   都在这一层），这正是 SCAN 域 P4 反复「扰动没被占据图看见」的在册主因（X23／X24）。换成机体长方体后，
//   实测口径的障碍保留从 3993 → 27214（×6.82，n 稿 §7.2 网格；小张 2026-10-11 逐格 1:1 复算过）。
//
// 四道判据的顺序**是承重面**（小张复核定档，缺一道就出问题）：
//   ① 非有限值：`!isfinite`。旧球形判据对 NaN 恒为假 ⇒ 放行；对 **Inf 判 `r² > blind²` 为真 ⇒ 一直保留**
//      （我原先写"球顺手删掉 NaN/Inf"是错的，Inf 从来没被删过；仿真实测 Inf=0 才没出事）。
//   ② 精确 (0,0,0)：仿真插件每帧发满点、无命中处填零，R82 实测占 74.6%。旧球靠 r=0 顺手删；换盒后它在盒外 ⇒
//      **不显式删就会把 ×5.84 的原点伪点灌进 /cloud_registered 与 SCAN 栅格**（沿机体轨迹堆出一坨自己造的障碍）。
//   ③ 机体长方体：吃**雷达系原始 xyz** ⇒ 不需 TF；雷达刚接躯干，正常/匍匐改的是离地高、不改盒在雷达系里的形状 ⇒ 对步态稳。
//   ④ 兜底小球：仍吃 `blind_`。yaml 里现取 **0.2**＝仿真侧严格 no-op（14 帧非零回波最小 r=0.2125），
//      真机侧语义只是"近原点杂波保险"，**不得写成"自遮挡保护到位"**（躯干/腿在 0.27–0.55m，0.2 罩不住）。
//
// H2 注记（X29 的实质，**这段只覆盖 C16 分支**）：
//   AviaHandler :71／Oust64Handler :99／VelodyneHandler :187 三处同形 `blind` 判据**一字未动**；
//   本链走 C16 是由 `faster_lio_outdoor.yaml:33 lidar_type: 4` 经 :19-38 的 switch 选定（分派另见 laser_mapping.cc）。
//   ⇒ 哪天 `lidar_type` 变成 1/2/3，本盒**静默不生效**、球形 blind 原样回来，**且没有任何门禁会红**。
//   在册实证这条"哪天"不是假设：`95_simulation/config/faster_lio_sim.yaml:39 lidar_type: 2`（由 `10_slam_mapping_sim.launch:8`
//   装载，G8sim 跑过那条链）⇒ 走那条路就是"旧球 0.2 还在、盒不在"的形态。生效与否由下面 Load() 打的 LOG 回执自证。
//
// 行号纪律（坑61）：本块**只追加在文件尾**、:208 一行改一行、此前所有行不增不删 ⇒
//   在册 `pointcloud_preprocess.cc:` 的 :19/:20/:57/:186/:187/:208 六组行号一律不位移；include 放此处（预处理按行处理，合法）正是为此。
// 默认安全：参数缺失/长度≠6/非有限 ⇒ **关盒、留兜底球、打一条 WARNING**（绝不把仿真约定值 0.38 焊进真机）。
// ============================================================================================
#include <cmath>
#include <ros/ros.h>
#include <vector>

namespace {

struct C16SelfMask {
    bool ready = false;
    bool box_on = false;
    bool drop_invalid = true;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    double hx = 0.0, hy = 0.0, hz = 0.0;

    void Load() {
        if (ready) return;
        ready = true;
        std::vector<double> box;
        if (!ros::param::get("self_mask_box", box) || box.size() != 6) {
            box_on = false;
            LOG(WARNING) << "[C13-乙] self_mask_box 缺失或长度!=6（实得 " << box.size()
                         << "）⇒ **关盒**、只留兜底球 blind（＝旧球形口径原样生效，本窗读数按旧口径读）";
            return;
        }
        for (size_t i = 0; i < 6; ++i) {
            if (!std::isfinite(box[i])) {
                box_on = false;
                LOG(WARNING) << "[C13-乙] self_mask_box[" << i << "] 非有限 ⇒ **关盒**、只留兜底球";
                return;
            }
        }
        cx = box[0]; cy = box[1]; cz = box[2];
        hx = box[3]; hy = box[4]; hz = box[5];
        double m = 0.0;
        if (!ros::param::get("self_mask_margin", m) || !std::isfinite(m) || m < 0.0) m = 0.0;
        hx += m; hy += m; hz += m;   // 外扩是各向同性的（真机腿部需求各向不同 ⇒ 均匀外扩表达不了，见 n 稿／q 稿 §0）
        int on_flag = 0;              // 默认 0＝关盒：读不到键时**不拿仿真的约定值当真机参数**
        ros::param::get("self_mask_box_on", on_flag);
        box_on = (on_flag != 0);
        bool di = true;
        if (ros::param::get("self_mask_drop_invalid", di)) drop_invalid = di;
        LOG(INFO) << "[C13-乙] 盒生效回执：box_on=" << box_on << " 心=(" << cx << "," << cy << "," << cz
                  << ") 半宽(含外扩)=(" << hx << "," << hy << "," << hz << ") 剔无效=" << drop_invalid
                  << " 兜底球 blind 由调用方传入 ⇒ **只覆盖 C16Handler**（H2）";
    }
};

C16SelfMask g_c16_mask;

}  // namespace

namespace faster_lio {

bool C16SelfMaskHit(double x, double y, double z, double range2, double blind) {
    g_c16_mask.Load();
    if (g_c16_mask.drop_invalid) {
        if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) return true;
        if (x == 0.0 && y == 0.0 && z == 0.0) return true;
    }
    const C16SelfMask &m = g_c16_mask;
    if (m.box_on &&
        std::fabs(x - m.cx) <= m.hx &&
        std::fabs(y - m.cy) <= m.hy &&
        std::fabs(z - m.cz) <= m.hz) {
        return true;
    }
    return range2 < (blind * blind);
}

}  // namespace faster_lio
