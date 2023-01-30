#!/usr/bin/env python3

import warnings
warnings.filterwarnings("ignore")


class Motor:
    def __init__(self, motor_id: int, ppr: int = 200):

        """
        Description:

        Args:
        """
        self.motor_id = motor_id
        self.ppr = ppr
        self.direction = False
        self.fault = False
        self.paused = False
        self.ramping = False
        self.enabled = False
        self.running = False
        self.sleep = False
        self.job_id = 0
        self.pulses_remaining = 0


        self.status_msg_job_id = 0
        self.command_job_id = 0
        self.job_in_progress = False
        self.job_status = 0
        self.first_status_received = False
        self.status_update_count = 0
        self.hardware_ready = False
