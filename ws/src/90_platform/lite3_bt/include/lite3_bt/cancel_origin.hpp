#ifndef LITE3_BT_CANCEL_ORIGIN_HPP
#define LITE3_BT_CANCEL_ORIGIN_HPP

#include <cstdint>

namespace lite3_bt
{
// Who asked the executor to stop?
//
// `/v8/local/cancel` has two legitimate writers: this BT (CancelStop and
// StopExecutor) and the global adapter's own fault stop.  Both end up as one
// `seq:outcome` frame on `/v8/local/cancel_ack`, and the ack carries no origin
// field, so the BT has to attribute it.  Numbering ranges are not a contract
// (the adapter could start anywhere, and a restart can reuse numbers), hence
// the BT remembers every seq it published itself.
//
// Provenance alone is not enough to conclude "the executor stopped on its
// own": three more facts have to agree (2026-11-22 review, item 6).
//   1. the outcome is `cancelled`   -- a refused/failed ack stopped nothing;
//   2. this exact seq was never seen before -- late and duplicate acks keep
//      arriving after the mission already acted on them;
//   3. a leg is still live (`has_current`) -- with no current goal there is
//      nothing to cancel, and a stale ack must not latch a cancel onto the
//      next mission.
// Only when all four hold does an ack mean "somebody else stopped the
// executor while we were driving", which is the one case the BT must turn
// into its own cancel instead of waiting for the arrival budget to expire.
struct CancelAckFacts
{
  std::uint64_t seq = 0;
  bool confirmed = false;        // outcome == "cancelled"
  bool self_issued = false;      // this BT published this seq
  bool already_recorded = false; // seq already resolved once
  bool has_current = false;      // a goal is being executed right now
};

enum class CancelAckDisposition
{
  RECORD_ONLY,    // keep the pre-existing behaviour: remember the outcome
  EXTERNAL_STOP,  // a cancel nobody here issued stopped a live leg
};

inline CancelAckDisposition classify_cancel_ack(const CancelAckFacts& facts)
{
  if (!facts.confirmed) return CancelAckDisposition::RECORD_ONLY;
  if (facts.self_issued) return CancelAckDisposition::RECORD_ONLY;
  if (facts.already_recorded) return CancelAckDisposition::RECORD_ONLY;
  if (!facts.has_current) return CancelAckDisposition::RECORD_ONLY;
  return CancelAckDisposition::EXTERNAL_STOP;
}
}  // namespace lite3_bt

#endif  // LITE3_BT_CANCEL_ORIGIN_HPP
