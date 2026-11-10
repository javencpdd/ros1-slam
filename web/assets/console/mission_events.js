/* Mission event bridge: turn /v8/bt/status into a mission decision.
   Pure logic, no DOM and no rosbridge dependency, so it can be unit-tested
   under node and shared by the browser console. */
(function (root, factory) {
  'use strict';
  var api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MissionEvents = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  var SUCCESS_EVENTS = ['reached'];
  var FAILURE_EVENTS = ['path_failed', 'path_timeout', 'timeout',
                        'planner_fail', 'recovered'];

  function parseStatus(text) {
    if (typeof text !== 'string' || !text.trim()) return null;
    var value;
    try { value = JSON.parse(text); } catch (e) { return null; }
    if (!value || typeof value !== 'object') return null;
    var seq = Number(value.current_seq);
    if (!isFinite(seq) || seq < 0) return null;
    return {
      event: typeof value.event === 'string' ? value.event : '',
      current_seq: seq,
      goal_seq: Number(value.goal_seq) || 0,
      queue_depth: Number(value.queue_depth) || 0,
      path_state: typeof value.path_state === 'string' ? value.path_state : ''
    };
  }

  /* Edge-triggered decision.
     sentSeq    = current_seq observed when this mission goal was published.
     handledSeq = current_seq whose terminal event was already consumed.
     A queued goal only counts once the BT has actually dequeued it
     (current_seq increased), and status messages repeat at 2 Hz, so both
     guards are needed to avoid a stale success or a double advance. */
  function classify(status, sentSeq, handledSeq) {
    if (!status) return { terminal: false, reason: 'no_status' };
    if (!(status.current_seq > sentSeq)) return { terminal: false, reason: 'stale_seq' };
    if (status.current_seq === handledSeq) return { terminal: false, reason: 'already_handled' };
    if (SUCCESS_EVENTS.indexOf(status.event) >= 0) {
      return { terminal: true, ok: true, reason: 'BT 判定到点', seq: status.current_seq };
    }
    if (FAILURE_EVENTS.indexOf(status.event) >= 0) {
      return { terminal: true, ok: false,
               reason: 'BT 事件：' + status.event, seq: status.current_seq };
    }
    return { terminal: false, reason: 'non_terminal' };
  }

  return {
    SUCCESS_EVENTS: SUCCESS_EVENTS,
    FAILURE_EVENTS: FAILURE_EVENTS,
    parseStatus: parseStatus,
    classify: classify
  };
});
