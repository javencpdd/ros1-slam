/* Offline tests for the /v8/bt/status mission decision.
   Run: node scripts/console/test_mission_events.js */
'use strict';

const fs = require('fs');
const path = require('path');
const M = require('./ui/assets/console/mission_events.js');

const ROOT = path.join(__dirname, '..');

let failures = 0;
function check(cond, what) {
  if (!cond) { failures += 1; console.log('FAIL  ' + what); }
  else console.log('PASS  ' + what);
}

function status(event, seq) {
  return JSON.stringify({ event: event, current_seq: seq, goal_seq: seq,
                          queue_depth: 0, path_state: 'ready' });
}

const parsed = M.parseStatus(status('reached', 3));
check(parsed && parsed.event === 'reached' && parsed.current_seq === 3,
      'valid status parses');
const withNodes = M.parseStatus(JSON.stringify({
  event: 'arrival_start', current_seq: 1, goal_seq: 1,
  planner_mode: 'signal', arrival_signal: true,
  nodes: [{ name: 'follow_path', status: 'RUNNING' }, { name: 7 }, null],
}));
check(withNodes && withNodes.nodes.length === 1 &&
      withNodes.nodes[0].name === 'follow_path' &&
      withNodes.nodes[0].status === 'RUNNING', 'bt nodes parsed and malformed dropped');
check(withNodes && withNodes.planner_mode === 'signal' &&
      withNodes.arrival_signal === true, 'planner mode and arrival flag parsed');
check(M.parseStatus('') === null, 'empty status refused');
check(M.parseStatus('{oops') === null, 'malformed JSON refused');
check(M.parseStatus('null') === null, 'JSON null refused');
check(M.parseStatus('{"event":"reached"}') === null, 'missing current_seq refused');
check(M.parseStatus('{"event":"reached","current_seq":-1}') === null,
      'negative seq refused');
check(M.parseStatus(null) === null, 'non-string refused');

// Main check: a fresh success advances the mission.
const main = M.classify(parsed, 2, null);
check(main.terminal === true && main.ok === true && main.seq === 3,
      'main: fresh reached terminal-success');

// Injection effectiveness: the verdict is not a constant green.
check(M.classify(M.parseStatus(status('dwell', 3)), 2, null).terminal === false,
      'non-terminal event does not advance');
check(M.classify(M.parseStatus(status('request_path', 3)), 2, null).terminal === false,
      'path-request event does not advance');

// Reverse control: without a seq increase nothing is decided.
check(M.classify(parsed, 3, null).reason === 'stale_seq',
      'reverse control: unchanged seq is stale');
check(M.classify(M.parseStatus(status('reached', 2)), 3, null).reason === 'stale_seq',
      'older seq cannot advance');

// Edge de-duplication: status repeats at 2 Hz.
check(M.classify(parsed, 2, 3).reason === 'already_handled',
      'repeated terminal status handled once');

// Failure events end the goal without a false success.
M.FAILURE_EVENTS.forEach((event) => {
  const v = M.classify(M.parseStatus(status(event, 4)), 3, null);
  check(v.terminal === true && v.ok === false, 'failure event ' + event + ' ends goal');
});

check(M.classify(null, 0, null).reason === 'no_status',
      'unparsable status never advances');
check(M.classify(M.parseStatus(status('reached', 1)), 0, null).terminal === true,
      'first goal (seq 0 -> 1) advances');

// Live-caught defect: BT overwrites event with dwell/stop inside one tick, so
// a 2 Hz status subscriber never sees "reached".  Persistent outcome must
// still drive the mission, while event-only status stays supported.
function outcomeStatus(outcome, value, event) {
  return JSON.stringify({ event: event || 'stop', current_seq: value,
                          goal_seq: value, outcome: outcome, outcome_seq: value });
}
const persisted = M.parseStatus(outcomeStatus('reached', 3));
check(persisted && persisted.outcome === 'reached' && persisted.outcome_seq === 3,
      'persistent outcome parses');
check(M.classify(persisted, 2, null).terminal === true &&
      M.classify(persisted, 2, null).ok === true,
      'outcome reached advances even when event is stop');
check(M.classify(M.parseStatus(outcomeStatus('failed', 4)), 3, null).ok === false,
      'outcome failed ends goal');
check(M.classify(persisted, 2, 3).reason === 'already_handled',
      'persistent outcome handled once');
check(M.classify(M.parseStatus(outcomeStatus('reached', 2)), 2, null).reason === 'stale_seq',
      'stale outcome does not advance');

// Wiring assertions: a green pure-function test is worthless if the browser
// never loads the module or onTopic never dispatches the status message.
const html = fs.readFileSync(path.join(ROOT, 'console/ui/console.html'), 'utf8');
const js = fs.readFileSync(path.join(ROOT, 'console/ui/assets/console/console.js'), 'utf8');
const iModule = html.indexOf('assets/console/mission_events.js');
const iMain = html.indexOf('assets/console/console.js');
check(iModule >= 0 && iMain >= 0 && iModule < iMain,
      'console.html loads mission_events before console.js');
check(js.indexOf("topic === '/v8/bt/status'") >= 0 &&
      js.indexOf('onBtStatus(msg)') >= 0,
      'onTopic dispatches /v8/bt/status to onBtStatus');
check(js.indexOf('window.MissionEvents.classify') >= 0 &&
      js.indexOf('mission.btSentSeq') >= 0,
      'mission uses the shared decision module');
check(html.indexOf('id="viz-backend"') >= 0 &&
      html.indexOf('id="foxglove-guide"') >= 0 &&
      html.indexOf('ros1_slam_sim_layout.json') >= 0,
      'console.html has viz backend switch and Foxglove guide');
check(js.indexOf('applyVizMode') >= 0 &&
      js.indexOf('/api/stacks/voxel_stream/stop') >= 0 &&
      js.indexOf("VIZ_HEAVY_TOPICS") >= 0,
      'console.js stops web-side load when Foxglove mode is chosen');
check(js.indexOf("mission.mode !== '3d'") >= 0,
      'pose-based arrival stays limited to 2D mode');

console.log((failures ? 'FAILED' : 'ALL PASS') + ' (' + failures + ' failures)');
process.exit(failures ? 1 : 0);
