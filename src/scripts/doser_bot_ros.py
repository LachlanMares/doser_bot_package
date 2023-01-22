#!/usr/bin/env python3

import warnings
import time
from pathlib import Path

# ROS Imports
import rospy

from doser_bot_package.msg import MotorStatus, DoserBotAcknowledge, DoserBotJobComplete, LoadCellStatus, DoserBotJobCancelled
from doser_bot_package.srv import EnableMotorService, DisableMotorService, CancelJobService, PauseJobService
from doser_bot_package.srv import ResumeJobService, WakeMotorService, SleepMotorService, ResetMotorService
from doser_bot_package.srv import MotorPulsesService, MotorPulsesSpecificRpmService
from doser_bot_package.srv import MotorRotationsService, MotorRotationsSpecificRpmService

from src import parse_definitions_file, DoserBot

warnings.filterwarnings("ignore")


def process_motor_rotations_specific_rpm_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    # print("pmrsr service")
    if new_request.motor in doserbot.motor_ids_in_use:
        return doserbot.send_motor_rotations_at_set_rpm(motor_id=new_request.motor,
                                                        number_or_rotations=new_request.rotations,
                                                        rpm=new_request.rpm,
                                                        direction=False if new_request.direction == 0 else True,
                                                        use_ramping=False if new_request.use_ramping == 0 else True,
                                                        ramping_steps=new_request.ramping_steps,
                                                        job_id=new_request.job_id,
                                                        )


def process_motor_pulses_specific_rpm_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_motor_pulses_at_set_rpm(motor_id=new_request.motor,
                                              rpm=new_request.rpm,
                                              pulses=new_request.pulses,
                                              direction=False if new_request.direction == 0 else True,
                                              use_ramping=False if new_request.use_ramping == 0 else True,
                                              ramping_steps=new_request.ramping_steps,
                                              job_id=new_request.job_id,
                                              )
        response = True

    return response


def process_motor_rotations_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_motor_rotations(motor_id=new_request.motor,
                                      number_or_rotations=new_request.rotations,
                                      direction=False if new_request.direction == 0 else True,
                                      microstep=new_request.microstep,
                                      pulse_interval=new_request.pulse_interval,
                                      pulse_on_period=new_request.pulse_on_period,
                                      use_ramping=False if new_request.use_ramping == 0 else True,
                                      ramping_steps=new_request.ramping_steps,
                                      job_id=new_request.job_id,
                                      )
        response = True

    return response


def process_motor_pulses_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_motor_pulses(motor_id=new_request.motor,
                                   direction=False if new_request.direction == 0 else True,
                                   microstep=new_request.microstep,
                                   pulses=new_request.pulses,
                                   pulse_interval=new_request.pulse_interval,
                                   pulse_on_period=new_request.pulse_on_period,
                                   use_ramping=False if new_request.use_ramping == 0 else True,
                                   ramping_steps=new_request.ramping_steps,
                                   job_id=new_request.job_id,
                                   )
        response = True

    return response


def process_motor_enable_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_enable_motor(motor_id=new_request.motor,)
        response = True

    return response


def process_motor_disable_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_disable_motor(motor_id=new_request.motor,)
        response = True

    return response


def process_motor_reset_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_reset_motor(motor_id=new_request.motor,)
        response = True

    return response


def process_motor_sleep_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_sleep_motor(motor_id=new_request.motor,)
        response = True

    return response


def process_motor_wake_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_wake_motor(motor_id=new_request.motor,)
        response = True

    return response


def process_pause_job_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_pause_job(motor_id=new_request.motor,)
        response = True

    return response


def process_resume_job_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_resume_job(motor_id=new_request.motor,)
        response = True

    return response


def process_cancel_job_service(new_request):
    """
    Description:
        ROS service.

    Args:
        new_request: Custom ROS service message

    Returns:
        response (boolean): Message valid or not
    """
    response = False

    if new_request.motor in doserbot.motor_ids_in_use:
        doserbot.send_cancel_job(motor_id=new_request.motor,)
        response = True

    return response


def process_response_messages(response_msg):
    ack_message = DoserBotAcknowledge()
    ack_message.header.stamp = rospy.get_rostime()
    ack_message.motor = response_msg[1]
    ack_message.command = response_msg[2]
    ack_message.response = response_msg[3]
    ack_message.confirmed = response_msg[4]
    return ack_message


def process_job_complete_messages(job_msg):
    job_message = DoserBotJobComplete()
    job_message.header.stamp = rospy.get_rostime()
    job_message.motor = job_msg[1]
    job_message.job_id = job_msg[2]
    return job_message


def process_job_cancelled_messages(job_msg):
    job_message = DoserBotJobCancelled()
    job_message.header.stamp = rospy.get_rostime()
    job_message.motor = job_msg[1]
    job_message.job_id = job_msg[2]
    return job_message


def publish_motor_status_message():
    for i, motor_id in enumerate(doserbot.motor_ids_in_use):
        motor_message = MotorStatus()
        motor_message.header.stamp = rospy.get_rostime()
        motor_message.motor = motor_id
        motor_message.status_bits = doserbot.motor_status_dict[f'motor_{motor_id}_status']
        motor_message.job_id = doserbot.motor_status_dict[f'motor_{motor_id}_job_id']
        motor_message.pulses_remaining = doserbot.motor_status_dict[f'motor_{motor_id}_pulses_remaining']
        motor_status_publisher.publish(motor_message)


def publish_load_cell_status_message():
    for i, load_cell_id in enumerate(doserbot.load_cell_ids_in_use):
        load_cell_message = LoadCellStatus()
        load_cell_message.header.stamp = rospy.get_rostime()
        load_cell_message.load_cell_id = load_cell_id
        load_cell_message.load_cell_grams = doserbot.load_cell_status_dict[f'load_cell_{load_cell_id}_value']
        load_cell_publisher.publish(load_cell_message)


def shutdown_hook():
    rospy.loginfo(f"doser-bot shutdown")


if __name__ == "__main__":
    desc = """Insert meaningful description here"""

    PROJECT_DIR = Path(__file__).resolve().parents[1]
    header_file = PROJECT_DIR / 'doser_bot_arduino/definitions.h'

    definitions_dict = parse_definitions_file(header_file)

    doserbot = DoserBot(definitions=definitions_dict, serial_port='/dev/ttyACM0')
    doserbot.start_thread()

    # Create a ROS node
    rospy.init_node("doser-bot-interface")
    rospy.on_shutdown(shutdown_hook)

    # Services(s)
    motor_rotations_specific_rpm_service_id = rospy.get_param(param_name="~motor_rotations_specific_rpm_service_name", default='motor_rotations_specific_rpm_service')
    motor_pulses_specific_rpm_service_id = rospy.get_param(param_name="~motor_pulses_specific_rpm_service_name", default='motor_pulses_specific_rpm_service')
    motor_rotations_service_id = rospy.get_param(param_name="~motor_rotations_service_name", default='motor_rotations_service')
    motor_pulses_service_id = rospy.get_param(param_name="~motor_pulses_service_name", default='motor_pulses_service')

    motor_enable_service_id = rospy.get_param(param_name="~motor_enable_service_name", default='motor_enable_service')
    motor_disable_service_id = rospy.get_param(param_name="~motor_disable_service_name", default='motor_disable_service')
    motor_sleep_service_id = rospy.get_param(param_name="~motor_sleep_service_name", default='motor_sleep_service')
    motor_wake_service_id = rospy.get_param(param_name="~motor_wake_service_name", default='motor_wake_service')
    motor_reset_service_id = rospy.get_param(param_name="~motor_reset_service_name", default='motor_reset_service')

    pause_job_service_id = rospy.get_param(param_name="~pause_job_service_name", default='pause_job_service')
    resume_job_service_id = rospy.get_param(param_name="~resume_job_service_name", default='resume_job_service')
    cancel_job_service_id = rospy.get_param(param_name="~cancel_job_service_name", default='cancel_job_service')

    # Publisher(s)
    motor_status_publisher_id = rospy.get_param(param_name="~motor_status_msg_id", default='motor_status')
    load_cell_publisher_id = rospy.get_param(param_name="~load_cell_status_msg_id", default='load_cell_status')
    command_acknowledge_publisher_id = rospy.get_param(param_name="~command_acknowledge_msg_id", default='command_acknowledge')
    job_complete_publisher_id = rospy.get_param(param_name="~job_complete_msg_id", default='job_complete')
    job_cancelled_publisher_id = rospy.get_param(param_name="~job_cancelled_msg_id", default='job_cancelled')

    motor_status_publisher = rospy.Publisher(motor_status_publisher_id, MotorStatus, queue_size=10)
    load_cell_publisher = rospy.Publisher(load_cell_publisher_id, LoadCellStatus, queue_size=1)
    command_acknowledge_publisher = rospy.Publisher(command_acknowledge_publisher_id, DoserBotAcknowledge, queue_size=1)
    job_complete_publisher = rospy.Publisher(job_complete_publisher_id, DoserBotJobComplete, queue_size=1)
    job_cancelled_publisher = rospy.Publisher(job_cancelled_publisher_id, DoserBotJobCancelled, queue_size=1)

    # Start the ROS services
    motor_rotations_specific_rpm_service = rospy.Service(motor_rotations_specific_rpm_service_id, MotorRotationsSpecificRpmService, process_motor_rotations_specific_rpm_service)
    motor_pulses_specific_rpm_service = rospy.Service(motor_pulses_specific_rpm_service_id, MotorPulsesSpecificRpmService, process_motor_pulses_specific_rpm_service)
    motor_rotations_service = rospy.Service(motor_rotations_service_id, MotorRotationsService, process_motor_rotations_service)
    motor_pulses_service = rospy.Service(motor_pulses_service_id, MotorPulsesService, process_motor_pulses_service)

    motor_enable_service = rospy.Service(motor_enable_service_id, EnableMotorService, process_motor_enable_service)
    motor_disable_service = rospy.Service(motor_disable_service_id, DisableMotorService, process_motor_disable_service)
    motor_sleep_service = rospy.Service(motor_sleep_service_id, SleepMotorService, process_motor_sleep_service)
    motor_wake_service = rospy.Service(motor_wake_service_id, WakeMotorService, process_motor_wake_service)
    motor_reset_service = rospy.Service(motor_reset_service_id, ResetMotorService, process_motor_reset_service)

    pause_job_serice = rospy.Service(pause_job_service_id, PauseJobService, process_pause_job_service)
    resume_job_service = rospy.Service(resume_job_service_id, ResumeJobService, process_resume_job_service)
    cancel_job_service = rospy.Service(cancel_job_service_id, CancelJobService, process_cancel_job_service)

    time.sleep(1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for response in doserbot.get_response_messages():
            command_acknowledge_publisher.publish(process_response_messages(response))

        for job in doserbot.get_job_complete_messages():
            job_complete_publisher.publish(process_job_complete_messages(job))

        for job in doserbot.get_job_cancelled_messages():
            job_cancelled_publisher.publish(process_job_cancelled_messages(job))

        publish_motor_status_message()
        publish_load_cell_status_message()

        rate.sleep()

    doserbot.stop()

SystemExit(0)
