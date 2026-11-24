"""真机迁移闸的两块纯逻辑：外参/阈值准入检查，以及定位运动的锁存状态机。

为什么要独立成件：
  ① 准入检查必须能"真的拦住启动"，而拦住的方式是进程非零退出 + roslaunch `required`
     把整窗带走。判据本身做成纯函数，才能用进程级用例证明"退出码非零"，而不是断言
     某段日志里出现了 WARN。
  ② 运动门控的要点不是"报警"而是**中止任务**：BT 的 `CheckLocalization` 只返回 FAILURE，
     而 `recovery_mode=skip` 与 `retry_max>=1` 会把失败目标重新排队 —— 定位一恢复就
     自动接着跑旧任务。本状态机在异常发生的那一刻发一次**取消**（清队），并且
     ★恢复后不自行发任何目标★；旧任务只能由操作者用新目标重新下达。

三条口径（对应 2026-11-24 的裁定）：
  - 外参未标定 ⇒ 真机导航不得启动；仿真域不走本件。
  - `quality_min == 0.0` 只是离线开发占位 ⇒ 正式真机必须给 G8 实测阈值。
  - 未知一律 fail-closed：读不到就当作不满足。
"""
import json
import os


class GateError(ValueError):
    """配置本身不合法（不是"未通过"，而是"没法判"）。"""


# ── 一、外参与阈值准入 ────────────────────────────────────────────────────
def load_attestation(path):
    """读标定记录；缺文件/坏 JSON 都返回 None（由调用方判未通过）。"""
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, encoding='utf-8') as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def check_extrinsic(attestation, extrinsic_xyz, quality_min,
                    allow_uncalibrated=False, require_attestation_fields=True,
                    require_calibrated=True):
    """判定真机导航是否可以启动。

    attestation：标定记录（dict）或 None
    extrinsic_xyz：launch/配置里当前生效的平移（3 元组）或 None
    quality_min：当前生效的内点下限
    require_calibrated：False 表示本入口**不能运动**（建图/定位干跑），只告警不阻断。
        这一档是必要的：标定数据本身要在真机上采，把采数入口一起堵死就成了死结。
        它必须由 launch 的 enable_motion 之类真值显式传入，不能默认开。
    返回 {'ok', 'reasons', 'degraded'}
    """
    reasons, degraded = [], []

    # 1) 外参：必须先有标定记录，且记录与现值一致
    if attestation is None:
        reasons.append('E_EXTRINSIC_UNATTESTED')
    else:
        got = attestation.get('xyz') or attestation.get('translation')
        if not got or len(got) < 3:
            reasons.append('E_EXTRINSIC_UNATTESTED')
        elif extrinsic_xyz is not None:
            try:
                if any(abs(float(got[i]) - float(extrinsic_xyz[i])) > 1e-6 for i in range(3)):
                    reasons.append('E_EXTRINSIC_MISMATCH')
            except (TypeError, ValueError):
                reasons.append('E_EXTRINSIC_MISMATCH')
        if require_attestation_fields:
            for field in ('measured_at', 'measured_by'):
                if not attestation.get(field):
                    reasons.append('E_EXTRINSIC_ATTESTATION_INCOMPLETE')

    # 2) 阈值：0.0 是离线开发占位，正式真机不得使用
    try:
        qmin = float(quality_min)
    except (TypeError, ValueError):
        qmin = 0.0
    if qmin <= 0.0:
        reasons.append('E_QUALITY_MIN_PLACEHOLDER')

    if not require_calibrated:
        # 不运动 ⇒ 不阻断，但必须标 degraded 并保留理由（日志要看得见）
        return {'ok': True, 'degraded': True, 'reasons': reasons,
                'not_required': True}

    ok = not reasons
    if reasons and allow_uncalibrated:
        # 显式开发口：只把"未标定"降级为 degraded，**阈值占位不降级**
        # —— 阈值占位会让质量判据形同虚设，那是另一类风险，不该被同一个开关带走。
        blocking = [r for r in reasons if not r.startswith('E_EXTRINSIC')]
        if not blocking:
            ok, degraded, reasons = True, list(reasons), []
    return {'ok': ok, 'degraded': bool(degraded), 'reasons': reasons}


# ── 二、定位运动门控 ─────────────────────────────────────────────────────
class MotionGate(object):
    """定位异常 → 中止任务并锁存；恢复后不自动重启旧任务。

    输出只有两类动作：`cancel`（清队）与 `note`（日志）。本闸**从不发目标**，
    所以"恢复后旧任务自己跑起来"在结构上不可能发生，除非操作者下新目标。
    """

    def __init__(self, republish_s=2.0, reason='loc_health'):
        if republish_s < 0:
            raise GateError('republish_s must be >= 0')
        self.republish_s = float(republish_s)
        self.reason = reason
        self.seq = 0
        self.latched = False
        self.failed_at = None
        self.last_cancel_at = None

    def update(self, healthy, now_s, detail=None):
        """返回动作列表：[{'type','topic_role','payload'}]，payload 为空表示不发。"""
        actions = []
        if not healthy:
            if not self.latched:
                self.latched = True
                self.failed_at = now_s
                self.seq += 1
                actions.append({'type': 'note',
                                'text': '定位异常，中止任务并锁存（detail=%s）' % (detail or '—')})
                actions.append({'type': 'cancel',
                                'payload': self._cancel_text(detail)})
                self.last_cancel_at = now_s
            elif self.republish_s and (now_s - self.last_cancel_at) >= self.republish_s:
                # 持续异常：定期重申取消，防止上游在我们之后又塞进来一次目标
                actions.append({'type': 'cancel', 'payload': self._cancel_text(detail)})
                self.last_cancel_at = now_s
            return actions

        if self.latched:
            # ★恢复不等于放行★：只解除"重复取消"，任务本身必须由操作者重新下达。
            self.latched = False
            actions.append({'type': 'note',
                            'text': '定位已恢复；旧任务不自动重启，等待新目标'})
        return actions

    def _cancel_text(self, detail):
        """BT 取消契约是文本 seq:scope:reason（JSON 会被判 malformed 丢掉）。"""
        return '%d:mission:%s' % (self.seq, self.reason if not detail else
                                  '%s_%s' % (self.reason, detail))
