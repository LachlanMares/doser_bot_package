#!/usr/bin/env python3

import rospy
import time

from doser_bot_package.msg import LoadCellStatus


class Motor:
    def __init__(self,
                 motor_id: int,
                 rotations_specific_rpm_service_id: str = 'motor_rotations_specific_rpm_service',
                 pulses_specific_rpm_service_id: str = 'motor_pulses_specific_rpm_service',
                 rotations_service_id: str = 'motor_rotations_service',
                 pulses_service_id: str = 'motor_pulses_service',
                 enable_service_id: str = 'motor_enable_service',
                 disable_service_id: str = 'motor_disable_service',
                 sleep_service_id: str = 'motor_sleep_service',
                 wake_service_id: str = 'motor_wake_service',
                 reset_service_id: str = 'motor_reset_service',
                 job_pause_service_id: str = 'pause_job_service',
                 job_resume_service_id: str = 'resume_job_service',
                 job_cancel_service_id: str = 'cancel_job_service',
                 wait_for_services: bool = False,
                 ppr: int = 200
                 ):

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
        self.status_msg_job_id = 0
        self.command_job_id = 0
        self.pulses_remaining = 0
        self.job_in_progress = False
        self.job_status = 0
        self.first_status_received = False
        self.status_update_count = 0
        self.hardware_ready = False

        # Services(s)
        self.motor_rotations_specific_rpm_service = rospy.ServiceProxy(rotations_specific_rpm_service_id, MotorRotationsSpecificRpmService)
        self.motor_pulses_specific_rpm_service = rospy.ServiceProxy(pulses_specific_rpm_service_id, MotorPulsesSpecificRpmService)
        self.motor_rotations_service = rospy.ServiceProxy(rotations_service_id, MotorRotationsService)
        self.motor_pulses_service = rospy.ServiceProxy(pulses_service_id, MotorPulsesService)

        self.motor_enable_service = rospy.ServiceProxy(enable_service_id, EnableMotorService)
        self.motor_disable_service = rospy.ServiceProxy(disable_service_id, DisableMotorService)
        self.motor_sleep_service = rospy.ServiceProxy(sleep_service_id, SleepMotorService)
        self.motor_wake_service = rospy.ServiceProxy(wake_service_id, WakeMotorService)
        self.motor_reset_service = rospy.ServiceProxy(reset_service_id, ResetMotorService)

        self.pause_job_service = rospy.ServiceProxy(job_pause_service_id, PauseJobService)
        self.resume_job_service = rospy.ServiceProxy(job_resume_service_id, ResumeJobService)
        self.cancel_job_service = rospy.ServiceProxy(job_cancel_service_id, CancelJobService)

        if wait_for_services:
            rospy.loginfo("waiting for doser-bot services")

            try:
                rospy.wait_for_service(rotations_specific_rpm_service_id, timeout=10.0)
                rospy.loginfo("wait no longer doser-bot services now active\n")

            except Exception as e:
                # Service call failed
                rospy.logerr(f"doser-bot service timeout: {e}\n")

    def is_motor_ready_for_new_job(self):
        """ """
        if self.first_status_received:
            if self.fault or self.sleep or not self.enabled:
                if self.fault:
                    self.motor_reset_service(motor=self.motor_id)
                    time.sleep(0.25)
                elif self.sleep:
                    self.motor_wake_service(motor=self.motor_id)
                    time.sleep(0.25)
                elif not self.enabled:
                    self.motor_enable_service(motor=self.motor_id)
                    time.sleep(0.25)
                self.hardware_ready = False

            else:
                self.hardware_ready = True

            if self.hardware_ready:
                if not self.running and self.job_status == 0:
                    return True

        return False

    def is_ready(self, units):
        if units > 0:
            self.hardware_ready = False
            return self.is_motor_ready_for_new_job(), True
        else:
             return True, False

    def enable(self):
        """ """
        self.motor_enable_service(motor=self.motor_id)

    def disable(self):
        """ """
        self.motor_disable_service(motor=self.motor_id)

    def reset(self):
        self.motor_reset_service(motor=self.motor_id)

    def motor_sleep(self):
        """ """
        self.motor_sleep_service(motor=self.motor_id)

    def wake(self):
        """ """
        self.motor_wake_service(motor=self.motor_id)

    def pause_job(self):
        """ """
        self.pause_job_service(motor=self.motor_id)

    def cancel_job(self):
        """ """
        self.cancel_job_service(motor=self.motor_id)

    def resume_job(self):
        """ """
        self.resume_job_service(motor=self.motor_id)

    def motor_rotations_at_rpm_from_message(self, data):
        """ """
        return self.motor_rotations_at_rpm(job_id=data.job_id,
                                           rotations=data.rotations,
                                           rpm=data.rpm,
                                           direction=False if data.direction == 0 else True,
                                           use_ramping=False if data.use_ramping == 0 else True,
                                           ramping_steps=data.ramping_steps)

    def motor_rotations_at_rpm(self, job_id: int, rotations: float, rpm: float, direction: bool,
                               use_ramping: bool = False, ramping_steps: int = 0):
        """ """
        if self.is_motor_ready_for_new_job():
            self.job_id = job_id
            service_response = self.motor_rotations_specific_rpm_service(motor=self.motor_id,
                                                                         rotations=rotations,
                                                                         rpm=rpm,
                                                                         direction=1 if direction else 0,
                                                                         use_ramping=1 if use_ramping else 0,
                                                                         ramping_steps=ramping_steps,
                                                                         job_id=job_id,
                                                                         )
            if service_response != -1:
                self.status_update_count = 3
                self.job_status = 1
                self.command_job_id = service_response
            return service_response

        else:
            return -1

    def motor_pulses_at_rpm_from_message(self, data):
        """ """
        return self.motor_pulses_at_rpm(job_id=data.job_id,
                                        pulses=data.pulses,
                                        rpm=data.rpm,
                                        direction=False if data.direction == 0 else True,
                                        use_ramping=False if data.use_ramping == 0 else True,
                                        ramping_steps=data.ramping_steps)

    def motor_pulses_at_rpm(self, job_id: int, pulses: int, rpm: float, direction: bool,
                            use_ramping: bool = False, ramping_steps: int = 0):
        """ """
        if self.is_motor_ready_for_new_job():
            service_response = self.motor_pulses_specific_rpm_service(motor=self.motor_id,
                                                                      pulses=pulses,
                                                                      rpm=rpm,
                                                                      direction=1 if direction else 0,
                                                                      use_ramping=1 if use_ramping else 0,
                                                                      ramping_steps=ramping_steps,
                                                                      job_id=job_id,
                                                                      )
            if service_response != -1:
                self.status_update_count = 3
                self.job_status = 1
                self.command_job_id = service_response

            return service_response

        else:
            return -1

    def motor_rotations_from_message(self, data):
        """ """
        return self.motor_rotations(job_id=data.job_id,
                                    rotations=data.rotations,
                                    microstep=data.microstep,
                                    direction=False if data.direction == 0 else True,
                                    use_ramping=False if data.use_ramping == 0 else True,
                                    ramping_steps=data.ramping_steps,
                                    pulse_interval=data.pulse_interval,
                                    pulse_on_period=data.pulse_on_period)

    def motor_rotations(self, job_id: int, rotations: int, direction: bool, use_ramping: bool = False, ramping_steps: int = 0,
                        microstep: int = 1, pulse_interval: int = 1500, pulse_on_period: int = 250):
        """ """
        if self.is_motor_ready_for_new_job():
            service_response = self.motor_rotations_service(motor=self.motor_id,
                                                            rotations=rotations,
                                                            microstep=microstep,
                                                            pulse_interval=pulse_interval,
                                                            pulse_on_period=pulse_on_period,
                                                            direction=1 if direction else 0,
                                                            use_ramping=1 if use_ramping else 0,
                                                            ramping_steps=ramping_steps,
                                                            job_id=job_id,
                                                            )

            if service_response != -1:
                self.status_update_count = 3
                self.job_status = 1
                self.command_job_id = service_response

            return service_response

        else:
            return -1

    def motor_pulses_from_message(self, data):
        """ """
        return self.motor_pulses(job_id=data.job_id,
                                 pulses=data.pulses,
                                 microstep=data.microstep,
                                 direction=False if data.direction == 0 else True,
                                 use_ramping=False if data.use_ramping == 0 else True,
                                 ramping_steps=data.ramping_steps,
                                 pulse_interval=data.pulse_interval,
                                 pulse_on_period=data.pulse_on_period)

    def motor_pulses(self, job_id: int, pulses: int, direction: bool, use_ramping: bool = False, ramping_steps: int = 0,
                     microstep: int = 1, pulse_interval: int = 1500, pulse_on_period: int = 250):
        """ """
        if self.is_motor_ready_for_new_job():
            service_response = self.motor_pulses_service(motor=self.motor_id,
                                                         pulses=pulses,
                                                         microstep=microstep,
                                                         pulse_interval=pulse_interval,
                                                         pulse_on_period=pulse_on_period,
                                                         direction=1 if direction else 0,
                                                         use_ramping=1 if use_ramping else 0,
                                                         ramping_steps=ramping_steps,
                                                         job_id=job_id,
                                                         )
            if service_response != -1:
                self.status_update_count = 3
                self.job_status = 1
                self.command_job_id = service_response

            return service_response

        else:
            return -1

    def job_complete(self, job_id):
        if self.job_id == job_id:
            self.job_status = 0
            self.job_id = 0
            self.motor_sleep()
            rospy.loginfo(f"job complete motor id: {self.motor_id}")

        else:
            rospy.loginfo(f"job complete id mis-match for motor id: {self.motor_id}")

    def job_cancelled(self, job_id):
        if self.job_id == job_id:
            self.job_status = 0
            self.job_id = 0
            self.motor_sleep()
            rospy.loginfo(f"job cancelled motor id: {self.motor_id}")

        else:
            rospy.loginfo(f"job cancelled id mis-match for motor id: {self.motor_id}")

    def job_abort(self):
        self.job_status = 0
        self.job_id = 0
        self.motor_sleep()
        rospy.loginfo(f"job aborted motor id: {self.motor_id}")
