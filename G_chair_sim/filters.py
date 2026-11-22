"""
filters.py — FlyPT 滤波函数仿真
所有函数定义源自 调教翻译.txt，用 Python 复现。
"""

import numpy as np


class EMALP:
    """指数移动平均 (EMALP: Exponential Moving Average)
    将周期内的数值平均化，消除毛刺/干扰。
    EMALP 是渐渐到达平均值。
    """
    def __init__(self, period_ms):
        self.period = period_ms / 1000.0  # 转为秒
        self.value = 0.0
        self.initialized = False

    def update(self, input_val, dt):
        alpha = dt / self.period if self.period > 0 else 1.0
        alpha = np.clip(alpha, 0, 1)
        if not self.initialized:
            self.value = input_val
            self.initialized = True
        else:
            self.value = (1 - alpha) * self.value + alpha * input_val
        return self.value


class LOGISTIC:
    """LOGISTIC: 无限接近式限制最大值
    曲线性接近，数值接近设定最大时输出增速变慢，最终无限接近限制值。
    output = max * tanh(input * k / max)
    """
    def __init__(self, max_val, k):
        self.max = max_val
        self.k = k

    def __call__(self, input_val):
        if self.max == 0:
            return 0
        return self.max * np.tanh(input_val * self.k / self.max)


class EDYNLP:
    """EDYNLP: 动态平滑滤波
    消除一定时间内的最大值，使输出变化不那么突然。
    作为速率限制器实现: 输出变化速率不超过 strength。
    strength 越大，平滑越强。
    """
    def __init__(self, period, strength):
        self.strength = strength  # 单位: 值/秒
        self.value = 0.0
        self.initialized = False

    def update(self, input_val, dt):
        if not self.initialized:
            self.value = input_val
            self.initialized = True
            return self.value
        max_change = self.strength * dt
        diff = input_val - self.value
        if np.abs(diff) <= max_change:
            self.value = input_val
        else:
            self.value += np.sign(diff) * max_change
        return self.value


class EMABS:
    """EMABS: 输出 = EMALP - EMABP (推荐)
    既有洗掉毛刺效果，又比纯 EMALP 延迟更低。
    """
    def __init__(self, period_ms):
        self.ema_pos = EMALP(period_ms)
        self.ema_neg = EMALP(period_ms)
        self.initialized = False

    def update(self, input_val, dt):
        pos = self.ema_pos.update(max(input_val, 0), dt)
        neg = self.ema_neg.update(max(-input_val, 0), dt)
        return pos - neg


class DEMALP:
    """双重指数移动平均 (DEMALP)
    先输出超过平均值再回到平均值。
    """
    def __init__(self, period_ms):
        self.ema1 = EMALP(period_ms)
        self.ema2 = EMALP(period_ms)

    def update(self, input_val, dt):
        e1 = self.ema1.update(input_val, dt)
        e2 = self.ema2.update(e1, dt)
        return 2.0 * e1 - e2


class CUBIC:
    """CUBIC: 四方函数
    output = A*x^3 + B*x^2 + C*x + D
    主要用于甩尾效果设定。
    """
    def __init__(self, a, b, c, d):
        self.a, self.b, self.c, self.d = a, b, c, d

    def __call__(self, x):
        return self.a * x**3 + self.b * x**2 + self.c * x + self.d


class FilterChain:
    """复合滤波链: 按 Mover 配置组合多个滤波器"""
    def __init__(self, config):
        self.cfg = config
        # 悬挂滤波: LOGISTIC(EMALP(VALUE;30);30;0.3)
        self.susp_emalp = EMALP(config['suspension']['emalp_period'])
        self.susp_logistic = LOGISTIC(
            config['suspension']['logistic_max'],
            config['suspension']['logistic_k']
        )
        # 旋转滤波: EDYNLP(VALUE;0;20)
        self.rot_edynlp = EDYNLP(
            config['rotation']['edynlp_period'],
            config['rotation']['edynlp_strength']
        )
        # 电缸输出滤波: LOGISTIC(VALUE;70;0.9)
        self.act_logistic = LOGISTIC(
            config['actuator']['logistic_max'],
            config['actuator']['logistic_k']
        )
        self.initialized = False

    def filter_suspension(self, input_val, dt):
        """悬挂滤波: EMALP → LOGISTIC"""
        smoothed = self.susp_emalp.update(input_val, dt)
        return self.susp_logistic(smoothed)

    def filter_rotation(self, input_val, dt):
        """旋转滤波: EDYNLP"""
        return self.rot_edynlp.update(input_val, dt)

    def filter_actuator(self, input_val):
        """电缸输出滤波: LOGISTIC"""
        return self.act_logistic(input_val)
