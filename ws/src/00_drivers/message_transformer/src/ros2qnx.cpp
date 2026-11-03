#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <ctime>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cerrno>
#include <string>
#include "message_transformer/SimpleCMD.h"
#include "message_transformer/ComplexCMD.h"
#include "../include/protocol.h"
#include "../include/sensor_logger.h"

using namespace std;

namespace {
// 官方 UDP 指令码（docs/protocol/lite3-udp-protocol.md 1.2.5 / 1.2.12 / 1.2.13）
const int CMD_HEARTBEAT = 0x21040001;
const int CMD_EXIT_AI = 0x2101052B;
const int CMD_MODE_MOVE = 0x21010D06;
const int CMD_GAIT_FLAT_LOW = 0x21010300;
const int CMD_GAIT_FLAT_MID = 0x21010307;
const int CMD_GAIT_FLAT_HIGH = 0x21010303;
const int CMD_GAIT_CRAWL = 0x21010406;
const int CMD_GAIT_GRIP = 0x21010402;
const int CMD_GAIT_GENERAL = 0x21010401;
const int CMD_GAIT_HIGH_STEP = 0x21010407;
}

static bool GetBoolParam(ros::NodeHandle& pnh, const std::string& name, bool fallback)
{
  XmlRpc::XmlRpcValue value;
  if (!pnh.getParam(name, value)) {
    ROS_WARN("[guard] param %s absent (ns=%s)", name.c_str(), pnh.getNamespace().c_str());
    return fallback;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    return static_cast<bool>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value) != 0;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
    const std::string text = static_cast<std::string>(value);
    return (text == "true" || text == "1" || text == "yes" || text == "on");
  }
  ROS_WARN("[guard] param %s has unsupported type %d", name.c_str(), value.getType());
  return fallback;
}

class ROS2QNX
{
public:
  ROS2QNX()
  {
    fd_ = socket(AF_INET, SOCK_DGRAM,0);
    if(fd_==-1){
      ROS_WARN("scoket create failed!");
    }

    addr_qnx_.sin_family = AF_INET;
    addr_qnx_.sin_port = htons(43893);
    addr_qnx_.sin_addr.s_addr = inet_addr("192.168.1.120");
  }

  ~ROS2QNX()
  {
    guard_enabled_ = false;
    if (heartbeat_thread_.joinable()) {
      heartbeat_thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  // 底盘守护：默认关闭；关闭时行为与本节点旧版完全一致（只转发速度）。
  void Configure(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");
    std::string target_ip = "192.168.1.120";
    int target_port = 43893;
    pnh.param<std::string>("guard_target_ip", target_ip, target_ip);
    pnh.param<int>("guard_target_port", target_port, target_port);
    addr_qnx_.sin_addr.s_addr = inet_addr(target_ip.c_str());
    addr_qnx_.sin_port = htons(target_port);

    guard_enabled_ = GetBoolParam(pnh, "chassis_guard_enable", false);
    auto_exit_ai_ = GetBoolParam(pnh, "guard_auto_exit_ai", true);
    auto_move_mode_ = GetBoolParam(pnh, "guard_auto_move_mode", true);
    dry_run_ = GetBoolParam(pnh, "guard_dry_run", false);
    pnh.param<double>("guard_heartbeat_hz", heartbeat_hz_, 2.0);
    pnh.param<double>("guard_gait_min_speed", gait_min_speed_, 0.10);
    std::string gait_topic = "/v8/chassis/gait";
    std::string status_topic = "/v8/chassis/status";
    pnh.param<std::string>("guard_gait_topic", gait_topic, gait_topic);
    pnh.param<std::string>("guard_status_topic", status_topic, status_topic);
    if (heartbeat_hz_ < 2.0) {
      ROS_WARN("[guard] heartbeat %.2f Hz < 2 Hz required by the protocol; clamped", heartbeat_hz_);
      heartbeat_hz_ = 2.0;
    }

    status_pub_ = nh.advertise<std_msgs::String>(status_topic, 5, true);
    gait_sub_ = nh.subscribe(gait_topic, 5, &ROS2QNX::GaitCallback, this);
    status_timer_ = nh.createTimer(ros::Duration(1.0), &ROS2QNX::StatusTimer, this);

    ROS_WARN("[guard] param chassis_guard_enable=%s auto_exit_ai=%s auto_move_mode=%s",
             guard_enabled_ ? "true" : "false", auto_exit_ai_ ? "true" : "false",
             auto_move_mode_ ? "true" : "false");
    if (!guard_enabled_) {
      ROS_INFO("[guard] disabled (chassis_guard_enable=false): velocity-only mode");
      return;
    }
    ROS_INFO("[guard] enabled: target=%s:%d hb=%.1fHz gait_min_speed=%.2f",
             target_ip.c_str(), target_port, heartbeat_hz_, gait_min_speed_);
    heartbeat_thread_ = std::thread(&ROS2QNX::HeartbeatLoop, this);
    ros::Duration(0.2).sleep();
    if (auto_exit_ai_) {
      SendSimple(CMD_EXIT_AI);
      ROS_INFO("[guard] sent exit-AI 0x%08X", CMD_EXIT_AI);
      ros::Duration(0.3).sleep();
    }
    if (auto_move_mode_) {
      SendSimple(CMD_MODE_MOVE);
      ROS_INFO("[guard] sent move-mode 0x%08X", CMD_MODE_MOVE);
    }
  }

  void CmdVelCallback(geometry_msgs::TwistConstPtr msg){
    int nbytes;
    last_speed_ = std::sqrt(msg->linear.x * msg->linear.x
                            + msg->linear.y * msg->linear.y);
    ComplexCMD complexcmd;
    complexcmd.cmd_code = 320;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = msg->linear.x;                  ///< linear x velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));

    complexcmd.cmd_code =325;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = msg->linear.y;                 ///< linear y velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));

    complexcmd.cmd_code = 321;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = -msg->angular.z;           ///< angular velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void KickBallCallback(std_msgs::Int32 msg){
    SimpleCMD cmd;
    cmd.cmd_code = 503;
    cmd.cmd_value = msg.data;
    cmd.type = 0;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void SimpleCMDCallback(message_transformer::SimpleCMD msg){
    SimpleCMD cmd;
    cmd.cmd_code = msg.cmd_code;
    cmd.cmd_value = msg.cmd_value;
    cmd.type = msg.type;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void ComplexCMDCallback(message_transformer::ComplexCMD msg){
    ComplexCMD cmd;
    cmd.cmd_code = msg.cmd_code;
    cmd.cmd_value = msg.cmd_value;
    cmd.type = msg.type;
    cmd.data = msg.data;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

private:
  int CodeForGait(const std::string& name) const
  {
    if (name == "flat_low") return CMD_GAIT_FLAT_LOW;
    if (name == "flat_mid") return CMD_GAIT_FLAT_MID;
    if (name == "flat_high") return CMD_GAIT_FLAT_HIGH;
    if (name == "crawl") return CMD_GAIT_CRAWL;
    if (name == "grip") return CMD_GAIT_GRIP;
    if (name == "general") return CMD_GAIT_GENERAL;
    if (name == "high_step") return CMD_GAIT_HIGH_STEP;
    return 0;
  }

  int SendSimple(int code, int value = 0, int type = 0)
  {
    SimpleCMD cmd;
    cmd.cmd_code = code;
    cmd.cmd_value = value;
    cmd.type = type;
    int sent = dry_run_ ? 0 : sendto(fd_, &cmd, sizeof(cmd), 0,
                                     (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
    if (!first_send_logged_) {
      first_send_logged_ = true;
      ROS_WARN("[guard] first send target=%s:%d fd=%d sent=%d errno=%d dry_run=%s",
               inet_ntoa(addr_qnx_.sin_addr), ntohs(addr_qnx_.sin_port), fd_,
               sent, errno, dry_run_ ? "true" : "false");
    }
    return sent;
  }

  // 导航层只发"切到哪种步态"；是否允许切由本节点按速度门限判定。
  void GaitCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (!guard_enabled_) {
      ROS_WARN_THROTTLE(5.0, "[guard] gait '%s' ignored: guard disabled", msg->data.c_str());
      return;
    }
    int code = CodeForGait(msg->data);
    if (code == 0) {
      ROS_WARN("[guard] unknown gait '%s'", msg->data.c_str());
      return;
    }
    if (last_speed_ > gait_min_speed_) {
      ROS_WARN("[guard] gait '%s' refused: speed %.3f > %.3f",
               msg->data.c_str(), last_speed_.load(), gait_min_speed_);
      last_refusal_ = "speed_too_high";
      return;
    }
    SendSimple(code);
    active_gait_ = msg->data;
    last_refusal_.clear();
    ROS_INFO("[guard] gait -> %s (0x%08X)", msg->data.c_str(), code);
  }

  void HeartbeatLoop()
  {
    const auto period = std::chrono::duration<double>(1.0 / heartbeat_hz_);
    auto next = std::chrono::steady_clock::now();
    while (ros::ok() && guard_enabled_) {
      SendSimple(CMD_HEARTBEAT);
      heartbeat_count_++;
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }

  void StatusTimer(const ros::TimerEvent&)
  {
    std::ostringstream out;
    out << "{\"enabled\":" << (guard_enabled_ ? "true" : "false")
        << ",\"gait\":\"" << active_gait_ << "\""
        << ",\"speed\":" << last_speed_.load()
        << ",\"heartbeat_count\":" << heartbeat_count_.load()
        << ",\"heartbeat_hz\":" << heartbeat_hz_
        << ",\"refusal\":\"" << last_refusal_ << "\"}";
    std_msgs::String msg;
    msg.data = out.str();
    status_pub_.publish(msg);
  }

  int fd_=-1;
  struct sockaddr_in addr_qnx_;
  std::atomic<bool> guard_enabled_{false};
  std::atomic<double> last_speed_{0.0};
  std::atomic<long> heartbeat_count_{0};
  bool auto_exit_ai_ = true;
  bool auto_move_mode_ = true;
  double heartbeat_hz_ = 2.0;
  double gait_min_speed_ = 0.10;
  std::string active_gait_ = "flat_low";
  std::string last_refusal_;
  bool first_send_logged_ = false;
  bool dry_run_ = false;
  std::thread heartbeat_thread_;
  ros::Publisher status_pub_;
  ros::Subscriber gait_sub_;
  ros::Timer status_timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle nh;

  ROS2QNX ros2qnx;
  ros2qnx.Configure(nh);
  ROS_INFO("-----   ros2qnx node up   -----");
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, &ROS2QNX::CmdVelCallback, &ros2qnx);
  ros::Subscriber vel_sub2 = nh.subscribe("cmd_vel_corrected", 1, &ROS2QNX::CmdVelCallback, &ros2qnx);
  ros::Subscriber kickball_sub = nh.subscribe("kick_ball", 1, &ROS2QNX::KickBallCallback, &ros2qnx);
  ros::Subscriber simplecmd_sub = nh.subscribe("simple_cmd", 1, &ROS2QNX::SimpleCMDCallback, &ros2qnx);
  ros::Subscriber complexcmd_sub = nh.subscribe("complex_cmd", 1, &ROS2QNX::ComplexCMDCallback, &ros2qnx);

  ros::spin();

  return 0;
}
