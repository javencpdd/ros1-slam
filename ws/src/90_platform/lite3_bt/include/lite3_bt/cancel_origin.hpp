#ifndef LITE3_BT_CANCEL_ORIGIN_HPP
#define LITE3_BT_CANCEL_ORIGIN_HPP

#include <cstdint>
#include <string>

namespace lite3_bt
{
// 取消信号的来路判定（2026-11-22 T-20261122-04 第二施工段）。
//
// 背景：`/v8/local/cancel` 不止一个写者。本节点（CancelStop／StopExecutor）会写，
// 全局适配器自诊断会写，操作员也可能直接写。执行层收到谁写的都照办，回一条
// `seq:outcome` 到 `/v8/local/cancel_ack`——回执里**没有来源字段**。
//
// 第一施工段的判据只看回执本身（已确认、号非自持、没记过账、手里有活腿），活体实测
// 暴露两个洞：腿在 mission 成功收尾时不清零，于是空闲期也「有活腿」；孤零零一条回执
// 与「真有人取消」在回执面上不可分，于是无关回执会误停。
//
// 这一段改成**同号三通道互证**，三条通道都是同一件事的独立投影：
//   甲 线上帧：`/v8/local/cancel` 上出现一条号不是本节点发的帧（帧本身就是那件事）；
//   乙 后端回显：`/global_path/status` 的 `E_FAR_CANCEL_RECEIVED` 带着它收到的 seq
//      （适配器只会在这个理由下把号带出来，见 far_node.on_cancel）；
//   丙 执行层回执：`/v8/local/cancel_ack` 的 `seq:cancelled`。
//
// 判定只用三件事，一件都不靠猜：
//   1. 这条 seq 是不是**本节点自己发的**——精确集合成员，不按号段猜来源；
//   2. 有没有活腿——任务关联用本节点自己的腿号（`current_seq`），不是时间量；
//   3. 这个号之前有没有别的通道作证——甲或乙任一即可，丙单独不作证。
//
// `IGNORE`／`REMEMBER`／`STOP_LEG` 三档语义：
//   IGNORE   ＝ 本节点自己发的（含代操作员转发），不处理；
//   REMEMBER ＝ 号陌生但此刻不能停（无活腿／同号已了结／孤证回执／已经在取消态），只记下来，
//              供后续同号通道互证，绝不落成取消意图；
//   STOP_LEG ＝ 有活腿且证据成立：把外来取消升格为本节点的取消，清队，退出 mission。
enum class OriginSignal
{
  IGNORE,
  REMEMBER,
  STOP_LEG,
};

// 甲／乙两路的输入事实（它们各自本身就是「有人发了取消帧」的证据）。
struct OriginFacts
{
  bool self_issued = false;       // seq ∈ 本节点已发集合
  bool leg_live = false;          // has_current：本节点正在执行一条腿
  bool already_accounted = false; // 该号已经触发过／已经了结
  bool already_latched = false;   // 本节点已经因为别的证据进入取消态
};

inline OriginSignal judge_origin_frame(const OriginFacts& facts)
{
  if (facts.self_issued) return OriginSignal::IGNORE;
  if (facts.already_latched) return OriginSignal::REMEMBER;
  if (facts.already_accounted) return OriginSignal::REMEMBER;
  return facts.leg_live ? OriginSignal::STOP_LEG : OriginSignal::REMEMBER;
}

// 丙路的输入事实。比第一施工段多的那一项是 `corroborated`：这个号必须在甲或乙上
// 见过，否则一条孤零零的回执不构成「有人取消」的证据（无关回执就是这一格）。
struct OriginAckFacts
{
  bool confirmed = false;         // outcome == "cancelled"
  bool self_issued = false;       // 本节点发过这个号
  bool already_recorded = false;  // 这个号已经了结（迟到／重复回执）
  bool corroborated = false;      // 甲或乙已经见过这个号
  bool leg_live = false;          // 手里还有活腿
  bool already_latched = false;   // 本节点已经因为别的证据进入取消态
};

inline OriginSignal judge_origin_ack(const OriginAckFacts& facts)
{
  if (!facts.confirmed) return OriginSignal::IGNORE;
  if (facts.self_issued) return OriginSignal::IGNORE;
  if (facts.already_latched) return OriginSignal::REMEMBER;
  if (facts.already_recorded) return OriginSignal::REMEMBER;
  if (!facts.corroborated) return OriginSignal::REMEMBER;
  return facts.leg_live ? OriginSignal::STOP_LEG : OriginSignal::REMEMBER;
}

// 从 `/global_path/status` 的 JSON 文本里取整数 seq 字段。只认 `"seq":` 后面紧跟的
// 十进制数字，取不到返回 false（缺字段／畸形帧不许当成 0 号去参与互证）。
inline bool parse_status_cancel_seq(const std::string& text, std::uint64_t* seq)
{
  if (seq == 0) return false;
  const std::string key = "\"seq\":";
  const std::size_t pos = text.find(key);
  if (pos == std::string::npos) return false;
  std::size_t i = pos + key.size();
  while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
  std::uint64_t value = 0;
  std::size_t digits = 0;
  while (i < text.size() && text[i] >= '0' && text[i] <= '9')
  {
    if (++digits > 19) return false;  // 溢出保护：uint64 最多 20 位
    value = value * 10 + static_cast<std::uint64_t>(text[i] - '0');
    ++i;
  }
  if (digits == 0) return false;
  *seq = value;
  return true;
}
}  // namespace lite3_bt

#endif  // LITE3_BT_CANCEL_ORIGIN_HPP
