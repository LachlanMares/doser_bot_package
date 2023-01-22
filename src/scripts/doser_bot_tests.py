#!/usr/bin/env python3

import warnings
import queue
import threading
import time
from typing import Union

from pathlib import Path

# ROS Imports
import rospy

from doser_bot_package.msg import MultiMotorPulsesAtRpmJob, MultiMotorRotationsAtRpmJob, SequentialMultiMotorRotationsAtRpmJob, MessageProgression
from doser_bot_package.msg import MultiMotorPulsesJob, MultiMotorRotationsJob, RotationsAtRpm

from src import parse_definitions_file

warnings.filterwarnings("ignore")


def process_message_progression(data):
    global ready_to_send
    info_string = f"job {data.message_id}, status {data.status}"

    if data.status == 3:
        info_string += f", motor {data.motor_id} in operation"
    rospy.loginfo(info_string)

    if data.status == 5 or data.status == 4:
        ready_to_send = True


def create_multi_motor_rotations_at_rpm_job(message_id: int,
                                            rpm: float,
                                            motor_ids: list,
                                            motor_rotations_list: list,
                                            direction: bool,
                                            use_ramping: bool = False,
                                            ramping_steps_list=None,
                                            sequential: bool = False,
                                            ):
    if sequential:
        rotations_job = SequentialMultiMotorRotationsAtRpmJob()
    else:
        rotations_job = MultiMotorRotationsAtRpmJob()

    rotations_job.header.stamp = rospy.get_rostime()
    rotations_job.message_id = message_id

    for j, motor_id in enumerate(motor_ids):
        motor = RotationsAtRpm()
        motor.motor_id = motor_id
        motor.direction = 1 if direction else 0
        motor.use_ramping = 1 if use_ramping else 0
        motor.job_id = message_id % 255
        motor.rpm = rpm
        motor.rotations = motor_rotations_list[j]
        motor.ramping_steps = ramping_steps_list[j] if use_ramping else 0
        rotations_job.__setattr__(f'motor_{j}', motor)

    return rotations_job


if __name__ == "__main__":
    desc = """Does some stuff to test the thing that needs testing"""

    PROJECT_DIR = Path(__file__).resolve().parents[1]
    header_file = PROJECT_DIR / 'definitions.h'

    definitions_dict = parse_definitions_file(definitions_filepath=header_file)

    motor_id_list = []
    for i in range(definitions_dict['motors']['NUMBER_OF_MOTORS']):
        motor_id_list.append(definitions_dict[f'motor_{i}']['id'])

    global ready_to_send

    # Create a ROS node
    rospy.init_node("doser_bot_test_node")

    multi_motor_pulses_id = rospy.get_param(param_name="~multi_motor_pulses_name", default='multi_motor_pulses')
    multi_motor_rotations_id = rospy.get_param(param_name="~multi_motor_rotations_name", default='multi_motor_rotations')
    multi_motor_pulses_at_rpm_id = rospy.get_param(param_name="~multi_motor_pulses_at_rpm_name", default='multi_motor_pulses_at_rpm')
    multi_motor_rotations_at_rpm_id = rospy.get_param(param_name="~multi_motor_rotations_at_rpm_name", default='multi_motor_rotations_at_rpm')
    sequential_multi_motor_rotations_at_rpm_id = rospy.get_param(param_name="~sequential_multi_motor_rotations_at_rpm_name", default='sequential_multi_motor_rotations_at_rpm')
    message_progression_status_id = rospy.get_param(param_name="~message_progression_status_name", default='message_progression_status')

    # Subscribers
    multi_motor_pulses_publisher = rospy.Publisher(multi_motor_pulses_id, MultiMotorPulsesJob, queue_size=1)
    multi_motor_rotations_publisher = rospy.Publisher(multi_motor_rotations_id, MultiMotorRotationsJob, queue_size=1)
    multi_motor_pulses_at_rpm_publisher = rospy.Publisher(multi_motor_pulses_at_rpm_id, MultiMotorPulsesAtRpmJob, queue_size=1)
    multi_motor_rotations_at_rpm_publisher = rospy.Publisher(multi_motor_rotations_at_rpm_id, MultiMotorRotationsAtRpmJob, queue_size=1)
    sequential_multi_motor_rotations_at_rpm_publisher = rospy.Publisher(sequential_multi_motor_rotations_at_rpm_id, SequentialMultiMotorRotationsAtRpmJob, queue_size=1)

    # Listener(s)
    rospy.Subscriber(message_progression_status_id, MessageProgression, process_message_progression, queue_size=2)
    time.sleep(2)
    sequential = False

    rotations_job = create_multi_motor_rotations_at_rpm_job(message_id=1,
                                                            rpm=200.0,
                                                            motor_ids=motor_id_list,
                                                            motor_rotations_list=[20]*10,
                                                            direction=False,
                                                            use_ramping=False,
                                                            sequential=sequential
                                                            )
    if sequential:
        sequential_multi_motor_rotations_at_rpm_publisher.publish(rotations_job)
    else:
        multi_motor_rotations_at_rpm_publisher.publish(rotations_job)

    ready_to_send = False

    # Time to get down to bidnezz
    while not rospy.is_shutdown():
        if ready_to_send:
            rospy.sleep(5)
            rotations_job.header.stamp = rospy.get_rostime()
            rotations_job.message_id += 1
            if sequential:
                sequential_multi_motor_rotations_at_rpm_publisher.publish(rotations_job)
            else:
                multi_motor_rotations_at_rpm_publisher.publish(rotations_job)
            ready_to_send = False

        rospy.sleep(0.1)

    SystemExit(0)
