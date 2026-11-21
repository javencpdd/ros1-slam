#!/usr/bin/env python3
"""local_planner 的控制侧：输出落地 + LocalControl 状态机（自 ros_node.py 拆出，M9）。

★为什么 publish_output 与 LocalControl 放同一模块★：AST 引用图显示
LocalControl -> publish_output。若 publish_output 留在 façade，就成了
ros_node ↔ ros_node_control 循环导入。
"""
import math
import sys
import time
from pathlib import Path

import json

import local_native as native
from contact_egress import ContactEgress, simulation_contact_egress
from ros_node_signals import (NON_LATCHING, feedback_at, feedback_uncertainty,
                              freshness, _local_pose_reason, stop_reason)

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / '90_platform/v8_platform'))
from common.pose_safety import (PoseError, PoseFreshness, finite_number, finite_vector,
                                map_to_body_planar, normalized_quaternion)

def finalize_output(out, orientation, body_enabled):
    """Decide both command streams before either is published."""
    try:
        if not isinstance(out, dict) or not isinstance(out.get('reason'), str):
            raise PoseError('E_LOCAL_OUTPUT')
        velocity = finite_vector(out['velocity'], 3, 'E_LOCAL_VELOCITY')
        if out.get('stop'):
            return dict(out, velocity=[0., 0., 0.]), (0., 0., 0.) if body_enabled else None
        body = map_to_body_planar(velocity, orientation) if body_enabled else None
        if body_enabled:
            velocity = (velocity[0], velocity[1], 0.)
        return dict(out, velocity=list(velocity)), body
    except (PoseError, ArithmeticError, KeyError, TypeError, ValueError) as error:
        reason = _local_pose_reason(getattr(error, 'code', 'E_LOCAL_VELOCITY'))
        return dict(velocity=[0., 0., 0.], stop=True, complete=False, reason=reason), (0., 0., 0.)

def publish_output(out, orientation, stamp, sequence, command_pub, twist_pub, messages):
    """Publish the same final decision; caller holds the callback/state lock."""
    stamped_type, twist_type, _ = messages
    out, body = finalize_output(out, orientation, twist_pub is not None)
    message = stamped_type()
    message.header.frame_id, message.header.stamp, message.header.seq = 'map', stamp, sequence
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = out['velocity']
    command_pub.publish(message)
    if twist_pub is not None:
        flat = twist_type()
        flat.linear.x, flat.linear.y, flat.linear.z = body
        twist_pub.publish(flat)
    return out

class LocalControl:
    """One control cycle, with a final atomic recheck against pose callbacks."""
    def __init__(self, shared, pose_input, lock, config, max_age, wall_watchdog_s,
                 command_pub, twist_pub, status_pub, messages, profile=None, sim_contact_egress=False,
                 gait_hold=None):
        self.shared, self.pose_input, self.lock = shared, pose_input, lock
        self.config, self.max_age, self.wall_watchdog_s = config, max_age, wall_watchdog_s
        self.command_pub, self.twist_pub, self.status_pub = command_pub, twist_pub, status_pub
        self.messages, self.profile = messages, profile
        self.gait_hold = gait_hold
        self.history, self.previous_stamp, self.previous_command = [], None, (0., 0., 0.)
        self.epoch, self.sequence = pose_input.guard.epoch, 0
        self.revocation = pose_input.revocation
        if type(sim_contact_egress) is not bool or (sim_contact_egress and (profile is not None or twist_pub is not None)):
            raise ValueError('contact egress is simulation-only without profile/plain Twist')
        self.sim_contact_egress = sim_contact_egress
        self._transfer_witness, self._egress, self._egress_key = None, None, None

    def _contact_key(self):
        return (self.shared.get('generation'), self.pose_input.guard.epoch, self.pose_input.revocation,
                self.shared.get('map', {}).get('map_id') if self.shared.get('map') else None,
                self.shared.get('map', {}).get('map_version') if self.shared.get('map') else None)

    def _clear_contact(self):
        if self._egress is not None:
            self._egress.retire()
        self._transfer_witness, self._egress, self._egress_key = None, None, None

    def _sync_contact(self):
        key, owner, map_value = self._contact_key(), self.shared.get('result'), self.shared.get('map')
        invalid = self._egress is not None and (self._egress_key != key
                  or not self._egress.valid_for(map_value, owner, self.config))
        witness = self._transfer_witness
        invalid = invalid or (witness is not None and (witness['key'] != key
                   or witness['owner'] is not owner or witness['map'] is not map_value))
        if invalid:
            self._clear_contact()

    def _contact_decision(self, out, ros_now, predicted, uncertainty):
        """Final locked decision; mutation happens only after actual publish."""
        raw, owner = self.shared['pose'], self.shared['result']
        if out['reason'].startswith('E_LOCAL_'):
            return out, ('clear', None)
        if self._egress is not None:
            clear = self._egress.full_clear(raw, predicted, self.previous_command, uncertainty)
            if out.get('complete') and not clear:
                return dict(velocity=[0., 0., 0.], stop=True, complete=False, reason='hold'), (None, None)
            return out, ('retire' if out.get('complete') and clear else 'close' if clear else None, None)
        witness = self._transfer_witness
        if out.get('complete') and witness is not None:
            geometry = ContactEgress.create(self.shared['map'], owner, self.config)
            if (geometry is not None and self.shared['pose_ros_stamp'] > witness['stamp']
                    and not geometry.full_clear(raw, predicted, self.previous_command, uncertainty)):
                # Recheck the original first leg on the current raw/pred/tail;
                # only its already accepted immediate next leg can follow.
                raw_support = native._support_at(self.shared['map'], raw, self.config)
                if (raw_support is not None and raw_support['surface_id'] == geometry.landed
                        and geometry.at_or_beyond(raw) and geometry.outside_old(raw)):
                    if (geometry.window(raw, predicted, self.previous_command, uncertainty,
                                        native._support_contains_segment) is not None):
                        return dict(out, complete=False, reason='hold'), ('activate', geometry)
                    # A terminal correction can leave the previous command's
                    # sub-micrometre transport tail behind the shared face for
                    # one tick. Keep the result and publish zero so the next
                    # window is evaluated without that tail.
                    return dict(out, complete=False, reason='hold'), (None, None)
        return out, (None, None)

    def _contact_published(self, out, action, predicted):
        kind, geometry = action
        if kind in ('clear', 'retire'):
            self._clear_contact()
            return
        if kind == 'close' and self._egress is not None:
            self._egress.close_permission()
        if kind == 'activate':
            self._egress, self._egress_key, self._transfer_witness = geometry, self._contact_key(), None
        if (self._egress is None and self._transfer_witness is None and not out['stop']
                and any(v != 0. for v in out['velocity'])):
            owner, map_value = self.shared['result'], self.shared['map']
            candidate = ContactEgress.create(map_value, owner, self.config)
            if candidate is not None:
                raw_support = native._support_at(map_value, self.shared['pose'], self.config)
                predicted_support = native._support_at(map_value, predicted, self.config)
                if (raw_support is not None and predicted_support is not None
                        and raw_support['surface_id'] == predicted_support['surface_id'] == candidate.source):
                    self._transfer_witness = dict(owner=owner, map=map_value, key=self._contact_key(),
                                                  stamp=self.shared['pose_ros_stamp'])

    def _reset_history(self):
        self.history.clear()
        self.previous_stamp, self.previous_command = None, (0., 0., 0.)
        self.epoch = self.pose_input.guard.epoch
        self.revocation = self.pose_input.revocation
        self._clear_contact()

    def tick(self, ros_clock, wall_clock, stamp_factory, clock_ready=True, clock_stamp=None):
        with self.lock:
            ros_now, wall_now = ros_clock(), wall_clock()
            self.pose_input.check(ros_now, wall_now)
            stamp_sec = ros_now if clock_stamp is None else clock_stamp
            if (not clock_ready or not finite_number(stamp_sec) or stamp_sec <= 0.
                    or stamp_sec > ros_now
                    or (self.shared.get('pose') is not None and stamp_sec < self.shared['pose_ros_stamp'])
                    or (self.previous_stamp is not None and stamp_sec < self.previous_stamp)):
                self.pose_input.invalidate('E_LOCAL_CLOCK')
                self._reset_history()
                stamp_sec = ros_now if finite_number(ros_now) and ros_now > 0. else 0.
            if (self.epoch != self.pose_input.guard.epoch
                    or self.revocation != self.pose_input.revocation):
                self._reset_history()
            if self.sim_contact_egress:
                self._sync_contact()
            contact = self._egress
            snapshot = dict(self.shared)
        fresh = snapshot.get('pose_error') or freshness(
            wall_now, snapshot['pose_at'], snapshot['obstacles_at'],
            snapshot['localization_at'], snapshot['localization_ok'], self.max_age)
        reason = stop_reason(fresh, snapshot['stop_latched'], snapshot['stop_reason'],
                             snapshot['result'], snapshot['result_at'], wall_now,
                             self.wall_watchdog_s, stamp_sec, snapshot['generation'],
                             snapshot['result_generation'], snapshot['last_error'])
        if not reason and snapshot['map'] is not None:
            try:
                estimated = feedback_at(snapshot['pose'], snapshot['pose_ros_stamp'], stamp_sec, self.history)
                period = (.05 if self.previous_stamp is None else max(.05, min(.1, stamp_sec - self.previous_stamp)))
                uncertainty = feedback_uncertainty(snapshot['pose_ros_stamp'], stamp_sec, self.history,
                                                   self.config.command_latency_bound_s)
                out = native.track_segment(snapshot['map'], snapshot['result'], estimated, stamp_sec,
                                           snapshot['obstacles'], config=self.config, control_period_s=period,
                                           feedback_pose=snapshot['pose'], uncertainty_m=uncertainty,
                                           previous_velocity=self.previous_command,
                                           **({'contact_egress': contact} if contact is not None else {}))
            except (ValueError, TypeError, KeyError, OverflowError) as error:
                out = dict(velocity=[0., 0., 0.], stop=True, reason='E_LOCAL_INPUT', input_error=str(error))
        else:
            out = dict(velocity=[0., 0., 0.], stop=True, reason=reason or snapshot['last_error'])
        # A pose/clock callback may invalidate the snapshot while tracking runs.
        # Recheck and publish under the SAME lock used by all those callbacks.
        with self.lock:
            latest_ros, latest_wall = ros_clock(), wall_clock()
            latest_status = self.pose_input.check(latest_ros, latest_wall)
            latest_fresh = self.shared.get('pose_error') or freshness(
                latest_wall, self.shared['pose_at'], self.shared['obstacles_at'],
                self.shared['localization_at'], self.shared['localization_ok'], self.max_age)
            if (self.epoch != self.pose_input.guard.epoch
                    or self.revocation != self.pose_input.revocation):
                self._reset_history()
                stamp_sec = latest_ros
            final_reason = stop_reason(
                latest_fresh, self.shared['stop_latched'], self.shared['stop_reason'],
                self.shared['result'], self.shared['result_at'], latest_wall,
                self.wall_watchdog_s, latest_ros, self.shared['generation'],
                self.shared['result_generation'], self.shared['last_error'])
            if (self.shared['generation'] != snapshot['generation']
                    or self.shared['result'] is not snapshot['result']):
                final_reason = latest_fresh or 'E_LOCAL_WORLD_UPDATED'
            contact_action = (None, None)
            contact_prediction = self.shared.get('pose')
            if self.sim_contact_egress:
                self._sync_contact()
                if not final_reason and self.shared.get('result') is not None:
                    try:
                        # New feedback may have arrived during the unlocked
                        # computation. Certify the CURRENT window under lock.
                        contact_prediction = feedback_at(self.shared['pose'], self.shared['pose_ros_stamp'],
                                                         latest_ros, self.history)
                        contact_uncertainty = feedback_uncertainty(self.shared['pose_ros_stamp'], latest_ros,
                                                                  self.history, self.config.command_latency_bound_s)
                        out = native.track_segment(self.shared['map'], self.shared['result'], contact_prediction,
                                                   latest_ros, self.shared['obstacles'], self.config,
                                                   control_period_s=period, feedback_pose=self.shared['pose'],
                                                   uncertainty_m=contact_uncertainty,
                                                   previous_velocity=self.previous_command,
                                                   **({'contact_egress': self._egress} if self._egress is not None else {}))
                        out, contact_action = self._contact_decision(out, latest_ros, contact_prediction,
                                                                     contact_uncertainty)
                    except (ValueError, TypeError, KeyError, OverflowError, native.LocalError):
                        out = dict(velocity=[0., 0., 0.], stop=True, reason='E_LOCAL_INPUT')
                        contact_action = ('clear', None)
                else:
                    contact_action = ('clear', None)
            if final_reason:
                out = dict(velocity=[0., 0., 0.], stop=True, reason=final_reason)
            # Gait changes happen only while stopped: hold zero from the desired
            # change until the chassis reports the new gait active and settled.
            # Timeout/stale keep the hold; a fail-closed reason above still wins.
            gait_state = None
            if self.gait_hold is not None:
                gait_state = self.gait_hold.update(
                    self.shared.get('gait_desired'),
                    self.shared.get('chassis_gait'),
                    self.shared.get('chassis_status_at'),
                    latest_wall)
                if gait_state['hold'] and not final_reason:
                    out = dict(velocity=[0., 0., 0.], stop=True, complete=False,
                               reason=gait_state['reason'], gait_hold=gait_state)
            # Header/history describe publication time, not the older tracking
            # sample. Lockstep's input tick only selects the calculation above.
            stamp_sec = latest_ros if finite_number(latest_ros) and latest_ros > 0. else 0.
            out = publish_output(out, self.shared.get('pose_orientation'), stamp_factory(stamp_sec),
                                 self.sequence, self.command_pub, self.twist_pub, self.messages)
            self.sequence += 1
            if self.sim_contact_egress:
                self._contact_published(out, contact_action, contact_prediction)
            self.history.append((stamp_sec, tuple(out['velocity'])))
            self.history[:] = self.history[-200:]
            self.previous_stamp, self.previous_command = stamp_sec, tuple(out['velocity'])
            latched = (self.shared['stop_latched'] or (out['reason'].startswith('E_LOCAL_')
                       and out['reason'] not in NON_LATCHING and snapshot['result_at'] > 0))
            status = dict(out, clock='ros_source+monotonic_progress', pose_clock=latest_status,
                          wall_watchdog_s=self.wall_watchdog_s,
                          plan_ms=snapshot.get('plan_ms'), detail=snapshot.get('last_error_detail'),
                          map_version=snapshot.get('map_version'), plan_ms_max=snapshot.get('plan_ms_max'),
                          latched=bool(latched), plain_twist_frame='base_link_yaw_plane' if self.twist_pub else None,
                          capability_domain='experimental_kinematic_carrier' if self.profile is None else 'validated_robot_profile')
            status['cancel'] = {'requested': bool(snapshot.get('cancel_requested')),
                                'seq': snapshot.get('cancel_seq'),
                                'reason': snapshot.get('cancel_reason')}
            # 诊断：三路输入的实时年龄（与 freshness 同一判据面），只在 freshness
            # 拒发时暴露哪个输入陈旧、陈旧多少。纯浮点运算，不引入新异常路径。
            if final_reason or reason:
                status['input_ages'] = {
                    'pose': round(latest_wall - self.shared['pose_at'], 3),
                    'obstacles': round(latest_wall - self.shared['obstacles_at'], 3),
                    'localization': round(latest_wall - self.shared['localization_at'], 3),
                    'obstacles_cb': self.shared.get('obstacles_cb', 0),
                    'max_age': self.max_age}
            if gait_state is not None:
                observed_at = self.shared.get('chassis_status_at')
                status['gait_hold'] = gait_state
                status['gait_observed'] = dict(
                    desired=self.shared.get('gait_desired'),
                    active=self.shared.get('chassis_gait'),
                    speed=self.shared.get('chassis_speed'),
                    status_age_s=(max(0., latest_wall - observed_at)
                                  if finite_number(observed_at) and observed_at > 0. else None))
            if self.sim_contact_egress and self._egress is not None:
                status['contact_egress'] = dict(source=self._egress.source, landed=self._egress.landed,
                                                segment_index=self._egress.index,
                                                permission_closed=self._egress.permission_closed,
                                                owner_created_at=self._egress.owner['created_at'])
            if isinstance(snapshot['result'], dict) and finite_number(snapshot['result'].get('valid_until')):
                status['plan_valid_until'] = snapshot['result']['valid_until']
            if self.shared.get('pose_clock_ordering') is not None:
                status['pose_clock_ordering'] = dict(self.shared['pose_clock_ordering'])
            self.status_pub.publish(self.messages[2](data=json.dumps(status)))
            if out.get('complete') or out['reason'].startswith('E_LOCAL_'):
                if self.shared['result'] is snapshot['result']:
                    terminal = self.shared['stop_latched'] or (latched and snapshot['result_at'] > 0)
                    terminal_reason = (self.shared['stop_reason'] if self.shared['stop_latched']
                                       else out['reason'])
                    self.shared.update(result=None, last_error=out['reason'], stop_latched=terminal,
                                       stop_reason=terminal_reason if terminal else None)
        return out
