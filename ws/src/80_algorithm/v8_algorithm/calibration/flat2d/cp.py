"""Clopper-Pearson 单侧 95% 下界（Lentz 连分式实现）。"""
import math


def betacf(a, b, x):
    MAXIT, EPS, FPMIN = 200, 3e-16, 1e-300
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    if abs(d) < FPMIN: d = FPMIN
    d = 1.0 / d
    h = d
    for m in range(1, MAXIT + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < FPMIN: d = FPMIN
        c = 1.0 + aa / c
        if abs(c) < FPMIN: c = FPMIN
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < FPMIN: d = FPMIN
        c = 1.0 + aa / c
        if abs(c) < FPMIN: c = FPMIN
        d = 1.0 / d
        de = d * c
        h *= de
        if abs(de - 1.0) < EPS:
            break
    return h


def betai(a, b, x):
    if x <= 0: return 0.0
    if x >= 1: return 1.0
    lbeta = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
    bt = math.exp(lbeta + a * math.log(x) + b * math.log(1.0 - x))
    if x < (a + 1.0) / (a + b + 2.0):
        return bt * betacf(a, b, x) / a
    return 1.0 - bt * betacf(b, a, 1.0 - x) / b


def cp_lower(k, n, alpha=0.05):
    """k 成功 / n 样本的单侧 Clopper-Pearson 下界。"""
    if k <= 0:
        return 0.0
    lo, hi = 0.0, 1.0
    for _ in range(80):
        mid = (lo + hi) / 2
        # P(X >= k | p) = I_p(k, n-k+1) = alpha 处即下界
        if betai(k, n - k + 1, mid) > alpha:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


if __name__ == '__main__':
    for k, n in ((24, 24), (36, 36), (29, 29), (18, 18), (20, 24), (23, 24)):
        print(k, n, round(cp_lower(k, n), 4))
