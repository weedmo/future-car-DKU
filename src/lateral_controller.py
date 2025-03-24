#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

# Lateral control class
# ICELAB (Han Sol Kim)

from typing import Any
import numpy as np
from PID import PID

class LateralController():
    def __init__(self, k_stanely: float=None, pid_angle: PID=None, pid_xpos: PID=None):
        self.k_stanely = k_stanely
        self.pid_angle = pid_angle
        self.pid_xpos = pid_xpos

    def stanely(self, xtarget: int, xpos: int, angle: int, vx: int) -> float:
        # xtarget:  target longitudinal position
        # xpos:     current longitudinal position
        # angle:    angular error
        # vx:       current longitudinal velocity        
        e = (xpos-xtarget)/480*0.85
        vx = vx/200*1.111
        delta = np.deg2rad(angle-90) + np.arctan2(self.k_stanely*e, vx)
        return -np.rad2deg(delta)

    def pid(self, xtarget: int, xpos: int, angle: int, vx: int) -> float:
        return self.pid_angle(90, angle) + self.pid_xpos(xtarget, xpos)
    
    def __call__(self, xtarget: int, xpos: int, angle: int, vx: int) -> float:
        return self.stanely(xtarget, xpos, angle, vx) if self.k_stanely is not None else \
               self.pid(xtarget, xpos, angle, vx)
    
    def get_pid_controller(kp:float=1, ki:float=0, kd:float=0, window_size:int=5, integral_limit:float=1.0):
        return PID(kp, ki, kd, window_size, integral_limit)