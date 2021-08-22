import time
import numpy as np


class Simple1DKalman:
    def __init__(self, err_measure, err_estimate=None, initial_estimate=0, q=0.01):
        """
        Args:
            err_measure: float: error that can be expected in the measurements (important)
            err_estimate: float (optional): error in the initial estimate (if None, same as err_measure)
            initial_estimate: float (optional): initial estimate
            q: float (optional): process variance (between 0 and 1, depending on how fast the process moves)
        """
        self._err_measure = err_measure
        self._err_estimate = err_estimate if err_estimate is not None else err_measure
        self._last_estimate = initial_estimate
        self._q = q

    def update_estimate(self, mea):
        kalman_gain = self._err_estimate / (self._err_estimate + self._err_measure)
        current_estimate = self._last_estimate + kalman_gain * (mea - self._last_estimate)
        self._err_estimate = (1.0 - kalman_gain) * self._err_estimate + self._q * abs(self._last_estimate - current_estimate)
        self._last_estimate = current_estimate
        return current_estimate


class Simple1DExponentialAverage:
    def __init__(self, tau, initial_estimate=0):
        """
        Args:
            tau: float: > 0, time constant of the filter
            initial_estimate: float (optional): initial estimate
        """
        self.tau = tau
        self._last_estimate = initial_estimate
        self._last_ts = time.monotonic()

    def _get_alpha(self):
        now = time.monotonic()
        dt = now - self._last_ts
        alpha = 1 - np.exp(- dt / self.tau)
        self._last_ts = now
        return alpha

    def update_estimate(self, mea):
        alpha = self._get_alpha()
        current_estimate = alpha * mea + (1 - alpha) * self._last_estimate
        self._last_estimate = current_estimate
        return current_estimate
