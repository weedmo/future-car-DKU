#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
######################################################
##                   PID Controller                 ##
##                                                  ##
##                ICELAB: Han Sol Kim               ##
######################################################

import numpy as np
class PID():
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, window_size=5, integral_limit=1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_int = 0

        self.error_history = np.zeros(window_size)
        self.integral_limit = integral_limit

    def __call__(self, r, x):
        error = r - x

        self.error_history = np.roll(self.error_history, -1)
        self.error_history[-1] = error

        error_diff = np.diff(self.error_history).mean()
        self.error_int += error
        self.error_int = np.clip(self.error_int, -self.integral_limit, self.integral_limit)

        p_term = self.kp * error
        i_term = self.ki * self.error_int
        d_term = self.kd * error_diff
                
        return p_term + i_term + d_term