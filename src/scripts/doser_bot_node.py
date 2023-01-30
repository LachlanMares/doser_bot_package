#!/usr/bin/env python3

import warnings
import time
import threading
from pathlib import Path

# ROS Imports
import rospy

from doser_bot_package.msg import MotorStatus, MultiMotorRotationsAtRpmJob, MessageProgression, DoserBotJobComplete, DoserBotJobCancelled

from src import parse_definitions_file, DoserBot, Motor

warnings.filterwarnings("ignore")


class MotorController:
    def __init__(self, definitions_dict: dict):

        """
        Description:

        Args:
        """
        self.doserbot = DoserBot(definitions=definitions_dict, serial_port='/dev/ttyACM0')
        self.doserbot.start_thread()

        self.motor_0 = Motor(motor_id=definitions_dict['motor_0']['id'], ppr=definitions_dict['motor_0']['steps_per_rev'])
        self.motor_1 = Motor(motor_id=definitions_dict['motor_1']['id'], ppr=definitions_dict['motor_1']['steps_per_rev'])
        self.motor_2 = Motor(motor_id=definitions_dict['motor_2']['id'], ppr=definitions_dict['motor_2']['steps_per_rev'])
        self.motor_3 = Motor(motor_id=definitions_dict['motor_3']['id'], ppr=definitions_dict['motor_3']['steps_per_rev'])
        self.motor_4 = Motor(motor_id=definitions_dict['motor_4']['id'], ppr=definitions_dict['motor_4']['steps_per_rev'])
        self.motor_5 = Motor(motor_id=definitions_dict['motor_5']['id'], ppr=definitions_dict['motor_5']['steps_per_rev'])
        self.motor_6 = Motor(motor_id=definitions_dict['motor_6']['id'], ppr=definitions_dict['motor_6']['steps_per_rev'])
        self.motor_7 = Motor(motor_id=definitions_dict['motor_7']['id'], ppr=definitions_dict['motor_7']['steps_per_rev'])
        self.motor_8 = Motor(motor_id=definitions_dict['motor_8']['id'], ppr=definitions_dict['motor_8']['steps_per_rev'])
        self.motor_9 = Motor(motor_id=definitions_dict['motor_9']['id'], ppr=definitions_dict['motor_9']['steps_per_rev'])

        # Timer(s)
        self.motor_status_timer = rospy.Timer(rospy.Duration(0.25), self.motor_status_timer_callback)
        self.check_for_messages_timer = rospy.Timer(rospy.Duration(0.1), self.check_for_messages)

        # Publishers(s)
        self.motor_status_publisher = rospy.Publisher(rospy.get_param(param_name="motor_status_msg_id", default='motor_status'), MotorStatus, queue_size=1)
        self.job_complete_publisher = rospy.Publisher(rospy.get_param(param_name="job_complete_msg_id", default='job_complete'), DoserBotJobComplete, queue_size=1)
        self.job_cancelled_publisher = rospy.Publisher(rospy.get_param(param_name="job_cancelled_msg_id", default='job_cancelled'), DoserBotJobCancelled, queue_size=1)

        message_progression_status_id = rospy.get_param(param_name="~message_progression_status_id",
                                                        default='message_progression_status')
        self.message_progression_status_publisher = rospy.Publisher(message_progression_status_id, MessageProgression, queue_size=1)

        # Subscriber(s)
        multi_motor_rotations_at_rpm_id = rospy.get_param(param_name="~multi_motor_rotations_at_rpm_id", default='/multi_motor_rotations_at_rpm')
        self.multi_motor_rotations_at_rpm_subscriber = rospy.Subscriber(multi_motor_rotations_at_rpm_id, MultiMotorRotationsAtRpmJob, self.multi_motor_rotations_at_rpm_callback, queue_size=1)

        self.doser_bot_messages_thread = threading.Thread(target=self.update, daemon=True)
        self.doser_bot_messages_thread.start()


    def multi_motor_rotations_at_rpm_callback(self, data):
        pass

    def motor_status_timer_callback(self):
        stamp = rospy.get_rostime()
        for i, motor_id in enumerate(self.doserbot.motor_ids_in_use):
            for motor in [self.motor_0, self.motor_1, self.motor_2, self.motor_3, self.motor_4, self.motor_5, self.motor_6, self.motor_7, self.motor_8, self.motor_9]:
                if motor_id == motor.motor_id:
                    motor_message = MotorStatus()
                    motor_message.header.stamp = stamp
                    motor_message.motor = motor_id
                    motor_message.status_bits = self.doserbot.motor_status_dict[f'motor_{motor_id}_status']
                    motor_message.job_id = self.doserbot.motor_status_dict[f'motor_{motor_id}_job_id']
                    motor_message.pulses_remaining = self.doserbot.motor_status_dict[f'motor_{motor_id}_pulses_remaining']
                    self.motor_status_publisher.publish(motor_message)

                    motor_status_dict = self.doserbot.decode_motor_status(motor_message.status_bits)
                    motor.direction = motor_status_dict["STATUS_DIRECTION_BIT"]
                    motor.fault = motor_status_dict["STATUS_FAULT_BIT"]
                    motor.paused = motor_status_dict["STATUS_PAUSED_BIT"]
                    motor.ramping = motor_status_dict["STATUS_RAMPING_BIT"]
                    motor.enabled = motor_status_dict["STATUS_ENABLED_BIT"]
                    motor.running = motor_status_dict["STATUS_RUNNING_BIT"]
                    motor.sleep = motor_status_dict["STATUS_SLEEP_BIT"]
                    motor.job_id = motor_message.job_id
                    motor.pulses_remaining = motor_message.pulses_remaining

    def enable(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_enable_motor(motor_id=motor_id)
            return True
        else:
            return False

    def disable(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_disable_motor(motor_id=motor_id)
            return True
        else:
            return False

    def reset(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_reset_motor(motor_id=motor_id)
            return True
        else:
            return False

    def motor_sleep(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_sleep_motor(motor_id=motor_id)
            return True
        else:
            return False

    def wake(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_wake_motor(motor_id=motor_id)
            return True
        else:
            return False

    def pause_job(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_pause_job(motor_id=motor_id)
            return True
        else:
            return False

    def cancel_job(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_cancel_job(motor_id=motor_id)
            return True
        else:
            return False

    def resume_job(self, motor_id):
        """ """
        if motor_id in self.doserbot.motor_ids_in_use:
            self.doserbot.send_resume_job(motor_id=motor_id)
            return True
        else:
            return False

    def process_response_messages(self, response_msg):
        for motor in [self.motor_0, self.motor_1, self.motor_2, self.motor_3, self.motor_4, self.motor_5, self.motor_6, self.motor_7, self.motor_8, self.motor_9]:
            if motor.motor_id == response_msg[1]:
                command = response_msg[2]
                response = response_msg[3]
                confirmed = response_msg[4]

    def process_job_complete_messages(self, job_msg):
        job_message = DoserBotJobComplete()
        job_message.header.stamp = rospy.get_rostime()
        job_message.motor = job_msg[1]
        job_message.job_id = job_msg[2]
        self.job_complete_publisher.publish(job_message)

    def process_job_cancelled_messages(self, job_msg):
        job_message = DoserBotJobCancelled()
        job_message.header.stamp = rospy.get_rostime()
        job_message.motor = job_msg[1]
        job_message.job_id = job_msg[2]
        self.job_cancelled_publisher.publish(job_message)

    def check_for_messages(self):
        for response in self.doserbot.get_response_messages():
            self.process_response_messages(response)

        for job in self.doserbot.get_job_complete_messages():
            self.process_job_complete_messages(job)

        for job in self.doserbot.get_job_cancelled_messages():
            self.process_job_cancelled_messages(job)


if __name__ == "__main__":
    desc = """Insert meaningful description here"""

    PROJECT_DIR = Path(__file__).resolve().parents[1]
    definitions_dict = parse_definitions_file(PROJECT_DIR / 'doser_bot_arduino/definitions.h')

    # Create a ROS node
    rospy.init_node("doser_bot_node")

    mc = MotorController(definitions_dict=definitions_dict)

    rospy.spin()
