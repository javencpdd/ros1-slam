"""真机定位健康判定：在"来源新鲜"之外，再加收敛、质量与异常跳变三条，未知一律 fail-closed。

为什么单独成件（不写进节点）：这三条判据最容易写成"没数据就当好"，而那正好是真机最危险的
情形——HDL 停止收敛或跳到别处时 TF 还在更新，只看新鲜度会一直报健康。做成纯函数后可以
用反控用例逐条证明"缺数据会翻红"，而不是靠读代码相信。

口径（三条，全部 fail-closed）：
  收敛  converged 话题新鲜且为 True；**没有配置该话题 = 未知 = 不健康**。
  质量  quality 话题新鲜且 >= quality_min；同样"没有 = 未知 = 不健康"。
  跳变  相邻两帧的位移/转角换算成速度，超过 max_plausible_mps 或超过单帧位移上限即判跳变。
        这一条不需要外部话题，纯靠位姿序列自证，因此任何配置下都生效。

唯一的降级口是 `allow_unverified=True`（显式、日志留痕）：把"未知"放行但仍然标记 degraded，
用于标定尚未接通的调试期。★默认关闭★，且它**不能**放行已判定为坏的样本（跳变/质量低/未收敛）。
"""

UNKNOWN_REASONS = (
    'E_LOC_QUALITY_UNKNOWN',
    'E_LOC_CONVERGED_UNKNOWN',
)


class HealthConfig(object):
    def __init__(self, max_plausible_mps=1.5, jump_max_m=0.50, jump_max_rad=0.60,
                 quality_min=0.0, quality_stale_s=1.0, converged_stale_s=1.0,
                 require_quality=True, require_converged=True, allow_unverified=False):
        self.max_plausible_mps = float(max_plausible_mps)
        self.jump_max_m = float(jump_max_m)
        self.jump_max_rad = float(jump_max_rad)
        self.quality_min = float(quality_min)
        self.quality_stale_s = float(quality_stale_s)
        self.converged_stale_s = float(converged_stale_s)
        self.require_quality = bool(require_quality)
        self.require_converged = bool(require_converged)
        self.allow_unverified = bool(allow_unverified)


def evaluate(pose_fresh, dt_s=None, delta_m=None, delta_rad=None,
             quality=None, quality_age_s=None,
             converged=None, converged_age_s=None, config=None):
    """返回 {'ok', 'degraded', 'reasons', 'checks'}；reasons 为空才 ok。"""
    cfg = config or HealthConfig()
    reasons = []
    checks = {}

    if not pose_fresh:
        reasons.append('E_LOC_NOT_FRESH')
    checks['fresh'] = bool(pose_fresh)

    # 跳变：纯自证项，任何配置下都判
    jumped = False
    if dt_s is not None and dt_s > 0. and delta_m is not None:
        speed = delta_m / dt_s
        checks['implied_speed_mps'] = round(speed, 3)
        if speed > cfg.max_plausible_mps or delta_m > cfg.jump_max_m:
            jumped = True
    if delta_rad is not None and delta_rad > cfg.jump_max_rad:
        jumped = True
    if jumped:
        reasons.append('E_LOC_JUMP')
    checks['jump'] = jumped

    # 收敛
    if cfg.require_converged:
        if converged is None or converged_age_s is None:
            reasons.append('E_LOC_CONVERGED_UNKNOWN')
            checks['converged'] = None
        elif converged_age_s > cfg.converged_stale_s:
            reasons.append('E_LOC_CONVERGED_UNKNOWN')
            checks['converged'] = None
        elif not converged:
            reasons.append('E_LOC_NOT_CONVERGED')
            checks['converged'] = False
        else:
            checks['converged'] = True
    else:
        checks['converged'] = 'not_required'

    # 质量
    if cfg.require_quality:
        if quality is None or quality_age_s is None:
            reasons.append('E_LOC_QUALITY_UNKNOWN')
            checks['quality'] = None
        elif quality_age_s > cfg.quality_stale_s:
            reasons.append('E_LOC_QUALITY_UNKNOWN')
            checks['quality'] = None
        elif quality < cfg.quality_min:
            reasons.append('E_LOC_QUALITY_LOW')
            checks['quality'] = round(quality, 4)
        else:
            checks['quality'] = round(quality, 4)
    else:
        checks['quality'] = 'not_required'

    degraded = False
    if reasons and cfg.allow_unverified:
        # 只允许放行"未知"，任何已判定的坏（跳变/未收敛/质量低/不新鲜）不得放行
        if all(r in UNKNOWN_REASONS for r in reasons):
            degraded = True
            reasons = []
    return {'ok': not reasons, 'degraded': degraded, 'reasons': reasons, 'checks': checks}
