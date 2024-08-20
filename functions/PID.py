#!/usr/bin/env python
# -*- coding: utf-8 -*-
class PID:
    """PID controller."""
    def __init__(self, Kp, Ki, Kd):
        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_error = 0.0

    def Update(self, error):
        # dt = random specified value
        # TODO: What is the effect of `dt`?
        # dt = 1.0 
        dt = 0.1 
        de = error - self.previous_error

        self.Cp = error
        self.Ci += error * dt
        self.Cd = de / dt

        self.previous_error = error

        return (
            (self.Kp * self.Cp)    # proportional term
            + (self.Ki * self.Ci)  # integral term
            + (self.Kd * self.Cd)  # derivative term
        )