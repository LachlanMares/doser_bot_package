#!/usr/bin/env python3

import warnings
import queue
import time

from pathlib import Path

# ROS Imports
import rospy

from doser_bot_package.msg import MotorStatus, DoserBotAcknowledge, DoserBotJobComplete, DoserBotJobCancelled
from doser_bot_package.msg import LoadCellStatus, MultiMotorPulsesAtRpmJob, MultiMotorRotationsAtRpmJob, MessageProgression
from doser_bot_package.msg import SequentialMultiMotorPulsesJob, SequentialMultiMotorRotationsJob
from doser_bot_package.msg import SequentialMultiMotorPulsesAtRpmJob, SequentialMultiMotorRotationsAtRpmJob
from doser_bot_package.msg import MultiMotorPulsesJob, MultiMotorRotationsJob
from doser_bot_package.srv import EnableMotorService, DisableMotorService, CancelJobService, PauseJobService
from doser_bot_package.srv import ResumeJobService, WakeMotorService, SleepMotorService, ResetMotorService
from doser_bot_package.srv import MotorPulsesService, MotorPulsesSpecificRpmService
from doser_bot_package.srv import MotorRotationsService, MotorRotationsSpecificRpmService

from src import parse_definitions_file

warnings.filterwarnings("ignore")

motor_job_queue = queue.Queue(maxsize=10)


class LoadCell:
    def __init__(self,
                 load_cell_id: int,
                 initial_value: int = 0,
                 minimum_value: int = -10000,
                 maximum_value: int = 10000,
                 load_cell_status_message_id: str = 'load_cell_status',
                 ):
        self.load_cell_id = load_cell_id
        self.current_weight = initial_value
        self.minimum_value = minimum_value
        self.maximum_value = maximum_value

        rospy.Subscriber(load_cell_status_message_id, LoadCellStatus, self.update_status, queue_size=2)

    def update_status(self, data):
        if data.load_cell_id == self.load_cell_id:
            if self.minimum_value <= data.load_cell_grams <= self.maximum_value:
                self.current_weight = data.load_cell_grams
            else:
                pass
                # rospy.loginfo(f"weight out of range for load_cell id: {self.load_cell_id}")


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


def motor_status_callback(data):
    for _motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8, motor_9]:
        if _motor.motor_id == data.motor:
            _motor.direction = (data.status_bits & 0b00000001) > 0
            _motor.fault = (data.status_bits & 0b00000010) > 0
            _motor.paused = (data.status_bits & 0b00000100) > 0
            _motor.ramping = (data.status_bits & 0b00001000) > 0
            # spare (status_message_data.status_bits & 0b00010000) > 0
            _motor.enabled = (data.status_bits & 0b00100000) > 0
            _motor.running = (data.status_bits & 0b01000000) > 0
            _motor.sleep = (data.status_bits & 0b10000000) > 0
            _motor.status_msg_job_id = data.job_id
            _motor.pulses_remaining = data.pulses_remaining

            if _motor.status_update_count > 0:
                _motor.status_update_count -= 1

            _motor.first_status_received = True


def command_acknowledge_callback(data):
    global ack
    for _motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8, motor_9]:
        if _motor.motor_id == data.motor:
            if data.confirmed == ack:
                # todo update based on response
                if data.command == _motor.command_job_id:
                    _motor.job_status = 2
                rospy.loginfo(f"acknowledge for motor id: {_motor.motor_id}, command {data.command}")

            else:
                if data.command == _motor.command_job_id:
                    _motor.job_status = 0
                rospy.loginfo(f"negative acknowledge for motor id: {_motor.motor_id}, command {data.command}, response {data.response}")


def job_complete_callback(data):
    for _motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8, motor_9]:
        if _motor.motor_id == data.motor:
            _motor.job_complete(job_id=data.job_id)


def job_cancelled_callback(data):
    for _motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8, motor_9]:
        if _motor.motor_id == data.motor:
            _motor.job_cancelled(job_id=data.job_id)


def multi_motor_rotations_callback(data):
    try:
        motor_job_queue.put(['mmr', data], block=False)
        rospy.loginfo("multi_motor_rotations")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def multi_motor_pulses_callback(data):
    try:
        motor_job_queue.put(['mmp', data], block=False)
        rospy.loginfo("multi_motor_pulses")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def multi_motor_rotations_at_rpm_callback(data):
    try:
        motor_job_queue.put(['mmrar', data], block=False)
        rospy.loginfo("multi_motor_rotations_at_rpm")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def multi_motor_pulses_at_rpm_callback(data):
    try:
        motor_job_queue.put(['mmpar', data], block=False)
        rospy.loginfo("multi_motor_pulses_at_rpm")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def sequential_multi_motor_rotations_callback(data):
    try:
        motor_job_queue.put(['smmr', data], block=False)
        rospy.loginfo("sequential_multi_motor_rotations")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def sequential_multi_motor_pulses_callback(data):
    try:
        motor_job_queue.put(['smmp', data], block=False)
        rospy.loginfo("sequential_multi_motor_pulses")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def sequential_multi_motor_rotations_at_rpm_callback(data):
    try:
        motor_job_queue.put(['smmrar', data], block=False)
        rospy.loginfo("sequential_multi_motor_rotations_at_rpm")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def sequential_multi_motor_pulses_at_rpm_callback(data):
    try:
        motor_job_queue.put(['smmpar', data], block=False)
        rospy.loginfo("sequential_multi_motor_pulses_at_rpm")

    except queue.Full:
        rospy.loginfo("motor job queue full")


def publish_progression_message(message_id: int, status: int, motor_id: int = 0):
    progression_message = MessageProgression()
    progression_message.header.stamp = rospy.get_rostime()
    progression_message.message_id = message_id
    progression_message.status = status
    progression_message.motor_id = motor_id
    message_progression_status_publisher.publish(progression_message)


if __name__ == "__main__":
    desc = """Does some motor stuff"""

    PROJECT_DIR = Path(__file__).resolve().parents[1]
    header_file = PROJECT_DIR / 'definitions.h'

    definitions_dict = parse_definitions_file(definitions_filepath=header_file)

    global ack

    # Create a ROS node
    rospy.init_node("doser_bot_motor_controller")

    # Parameters
    motor_status_message_id = rospy.get_param(param_name="~motor_status_msg_id", default='motor_status')
    command_acknowledge_message_id = rospy.get_param(param_name="~command_acknowledge_msg_id", default='command_acknowledge')
    job_complete_message_id = rospy.get_param(param_name="~job_complete_msg_id", default='job_complete')
    job_cancelled_message_id = rospy.get_param(param_name="~job_cancelled_msg_id", default='job_cancelled')

    multi_motor_pulses_id = rospy.get_param(param_name="~multi_motor_pulses_name", default='multi_motor_pulses')
    multi_motor_rotations_id = rospy.get_param(param_name="~multi_motor_rotations_name", default='multi_motor_rotations')
    multi_motor_pulses_at_rpm_id = rospy.get_param(param_name="~multi_motor_pulses_at_rpm_name", default='multi_motor_pulses_at_rpm')
    multi_motor_rotations_at_rpm_id = rospy.get_param(param_name="~multi_motor_rotations_at_rpm_name", default='multi_motor_rotations_at_rpm')
    sequential_multi_motor_pulses_id = rospy.get_param(param_name="~sequential_multi_motor_pulses_name", default='sequential_multi_motor_pulses')
    sequential_multi_motor_rotations_id = rospy.get_param(param_name="~sequential_multi_motor_rotations_name", default='sequential_multi_motor_rotations')
    sequential_multi_motor_pulses_at_rpm_id = rospy.get_param(param_name="~sequential_multi_motor_pulses_at_rpm_name", default='sequential_multi_motor_pulses_at_rpm')
    sequential_multi_motor_rotations_at_rpm_id = rospy.get_param(param_name="~sequential_multi_motor_rotations_at_rpm_name", default='sequential_multi_motor_rotations_at_rpm')
    message_progression_status_id = rospy.get_param(param_name="~message_progression_status_name", default='message_progression_status')

    # Subscribers(s)
    rospy.Subscriber(motor_status_message_id, MotorStatus, motor_status_callback, queue_size=50)
    rospy.Subscriber(command_acknowledge_message_id, DoserBotAcknowledge, command_acknowledge_callback, queue_size=20)
    rospy.Subscriber(job_complete_message_id, DoserBotJobComplete, job_complete_callback, queue_size=20)
    rospy.Subscriber(job_cancelled_message_id, DoserBotJobCancelled, job_cancelled_callback, queue_size=20)

    rospy.Subscriber(multi_motor_pulses_id, MultiMotorPulsesJob, multi_motor_pulses_callback, queue_size=1)
    rospy.Subscriber(multi_motor_rotations_id, MultiMotorRotationsJob, multi_motor_rotations_callback, queue_size=1)
    rospy.Subscriber(multi_motor_pulses_at_rpm_id, MultiMotorPulsesAtRpmJob, multi_motor_pulses_at_rpm_callback, queue_size=1)
    rospy.Subscriber(multi_motor_rotations_at_rpm_id, MultiMotorRotationsAtRpmJob, multi_motor_rotations_at_rpm_callback, queue_size=1)
    rospy.Subscriber(sequential_multi_motor_pulses_id, SequentialMultiMotorPulsesJob, sequential_multi_motor_pulses_callback, queue_size=1)
    rospy.Subscriber(sequential_multi_motor_rotations_id, SequentialMultiMotorRotationsJob, sequential_multi_motor_rotations_callback, queue_size=1)
    rospy.Subscriber(sequential_multi_motor_pulses_at_rpm_id, SequentialMultiMotorPulsesAtRpmJob, sequential_multi_motor_pulses_at_rpm_callback, queue_size=1)
    rospy.Subscriber(sequential_multi_motor_rotations_at_rpm_id, SequentialMultiMotorRotationsAtRpmJob, sequential_multi_motor_rotations_at_rpm_callback, queue_size=1)

    # Publishers
    message_progression_status_publisher = rospy.Publisher(message_progression_status_id, MessageProgression, queue_size=1)

    load_cell_0 = LoadCell(load_cell_id=definitions_dict['load_cell_0']['id'], initial_value=0)

    motor_0 = Motor(motor_id=definitions_dict['motor_0']['id'], ppr=definitions_dict['motor_0']['steps_per_rev'])
    motor_1 = Motor(motor_id=definitions_dict['motor_1']['id'], ppr=definitions_dict['motor_1']['steps_per_rev'])
    motor_2 = Motor(motor_id=definitions_dict['motor_2']['id'], ppr=definitions_dict['motor_2']['steps_per_rev'])
    motor_3 = Motor(motor_id=definitions_dict['motor_3']['id'], ppr=definitions_dict['motor_3']['steps_per_rev'])
    motor_4 = Motor(motor_id=definitions_dict['motor_4']['id'], ppr=definitions_dict['motor_4']['steps_per_rev'])
    motor_5 = Motor(motor_id=definitions_dict['motor_5']['id'], ppr=definitions_dict['motor_5']['steps_per_rev'])
    motor_6 = Motor(motor_id=definitions_dict['motor_6']['id'], ppr=definitions_dict['motor_6']['steps_per_rev'])
    motor_7 = Motor(motor_id=definitions_dict['motor_7']['id'], ppr=definitions_dict['motor_7']['steps_per_rev'])
    motor_8 = Motor(motor_id=definitions_dict['motor_8']['id'], ppr=definitions_dict['motor_8']['steps_per_rev'])
    motor_9 = Motor(motor_id=definitions_dict['motor_9']['id'], ppr=definitions_dict['motor_9']['steps_per_rev'], wait_for_services=True)

    job_id = 0
    ack = definitions_dict['serial_settings']['ACK']

    # Time to get down to bidnezz
    while not rospy.is_shutdown():
        if not motor_job_queue.empty():

            queue_data = motor_job_queue.get(timeout=0.1)

            publish_progression_message(message_id=queue_data[1].message_id, status=0)
            all_required_motors_ready = False
            expected_time_required = [0] * definitions_dict['motors']['NUMBER_OF_MOTORS']

            while not all_required_motors_ready:
                all_required_motors_ready = True

                for i, motor_info in enumerate([queue_data[1].motor_0, queue_data[1].motor_1, queue_data[1].motor_2,
                                                queue_data[1].motor_3, queue_data[1].motor_4, queue_data[1].motor_5,
                                                queue_data[1].motor_6, queue_data[1].motor_7, queue_data[1].motor_8,
                                                queue_data[1].motor_9]):
                    for motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8, motor_9]:

                        if motor.motor_id == motor_info.motor_id:
                            if queue_data[0] in ['mmrar', 'smmrar']:
                                ready, required = motor.is_ready(units=motor_info.rotations)
                                expected_time_required[i] = motor_info.rotations * (60 / motor_info.rpm)  # rotations * revs per second

                            elif queue_data[0] in ['mmr', 'smmr']:
                                ready, required = motor.is_ready(units=motor_info.rotations)
                                pulses = motor_info.rotations * motor.ppr * motor_info.microstep
                                expected_time_required[i] = (motor_info.pulse_interval / 10e6) * pulses  # pulses per second * pulses

                            elif queue_data[0] in ['mmpar', 'smmpar']:
                                ready, required = motor.is_ready(units=motor_info.pulses)
                                expected_time_required[i] = (motor_info.pulses / motor.ppr) * (60 / motor_info.rpm)  # rotations * revs per second

                            elif queue_data[0] in ['mmp', 'smmp']:
                                ready, required = motor.is_ready(units=motor_info.pulses)
                                expected_time_required[i] = (motor_info.pulse_interval / 10e6) * motor_info.pulses  # pulses * pulses per second

                            else:
                                ready, required = True, False
                                expected_time_required[i] = 0

                            if not ready and required:
                                all_required_motors_ready = False

                rospy.sleep(0.2)

            if queue_data[0] in ['mmr', 'mmrar', 'mmpar', 'mmp']:
                required_job_time = max(expected_time_required)
            elif queue_data[0] in ['smmr', 'smmrar', 'smmpar', 'smmp']:
                required_job_time = sum(expected_time_required)
            else:
                required_job_time = 0

            publish_progression_message(message_id=queue_data[1].message_id, status=1)

            if queue_data[0] in ['mmr', 'mmrar', 'mmpar', 'mmp']:
                for motor_info in [queue_data[1].motor_0, queue_data[1].motor_1, queue_data[1].motor_2,
                                   queue_data[1].motor_3, queue_data[1].motor_4, queue_data[1].motor_5,
                                   queue_data[1].motor_6, queue_data[1].motor_7, queue_data[1].motor_8,
                                   queue_data[1].motor_9]:
                    for motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8,
                                  motor_9]:

                        if motor.motor_id == motor_info.motor_id and motor_info.rotations > 0:
                            if queue_data[0] in ['mmr']:
                                if motor.motor_rotations_from_message(data=motor_info) == -1:
                                    rospy.loginfo(f"error assigning job {queue_data[1].message_id} to motor id: {motor.motor_id}")

                            elif queue_data[0] in ['mmp']:
                                if motor.motor_pulses_from_message(data=motor_info) == -1:
                                    rospy.loginfo(f"error assigning job {queue_data[1].message_id} to motor id: {motor.motor_id}")

                            elif queue_data[0] in ['mmrar']:
                                if motor.motor_rotations_at_rpm_from_message(data=motor_info) == -1:
                                    rospy.loginfo(f"error assigning job {queue_data[1].message_id} to motor id: {motor.motor_id}")

                            elif queue_data[0] in ['mmpar']:
                                if motor.motor_pulses_at_rpm_from_message(data=motor_info) == -1:
                                    rospy.loginfo(f"error assigning job {queue_data[1].message_id} to motor id: {motor.motor_id}")

                publish_progression_message(message_id=queue_data[1].message_id, status=2)

                all_required_motors_complete = False

                start_time = time.time()
                timeout_flag = False
                error_flag = False

                while not all_required_motors_complete:
                    all_required_motors_complete = True

                    for motor_info in [queue_data[1].motor_0, queue_data[1].motor_1, queue_data[1].motor_2,
                                       queue_data[1].motor_3, queue_data[1].motor_4, queue_data[1].motor_5,
                                       queue_data[1].motor_6, queue_data[1].motor_7, queue_data[1].motor_8,
                                       queue_data[1].motor_9]:
                        for motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7,
                                      motor_8, motor_9]:

                            if motor.motor_id == motor_info.motor_id and motor_info.rotations > 0:
                                if motor.job_status != 0:
                                    if motor.pulses_remaining == 0 and not motor.running and motor.status_update_count == 0 and timeout_flag:
                                        motor.job_complete(motor.job_id)
                                    all_required_motors_complete = False

                    if time.time() - start_time > required_job_time + 1:
                        timeout_flag = True
                    elif time.time() - start_time > required_job_time + 3:
                        rospy.logerr(f"job id {queue_data[1].message_id} exceeded maximum time, motor(s) not complete")
                        all_required_motors_complete = True
                        error_flag = True

            elif queue_data[0] in ['smmr', 'smmrar', 'smmpar', 'smmp']:

                start_time = time.time()
                error_flag = False

                for i, motor_info in enumerate([queue_data[1].motor_0, queue_data[1].motor_1, queue_data[1].motor_2,
                                                queue_data[1].motor_3, queue_data[1].motor_4, queue_data[1].motor_5,
                                                queue_data[1].motor_6, queue_data[1].motor_7, queue_data[1].motor_8,
                                                queue_data[1].motor_9]):
                    if not error_flag:
                        for motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7, motor_8, motor_9]:

                            if motor.motor_id == motor_info.motor_id and motor_info.rotations > 0:

                                if queue_data[0] in ['smmpar']:
                                    job_ok = motor.motor_pulses_at_rpm_from_message(data=motor_info)
                                elif queue_data[0] in ['smmrar']:
                                    job_ok = motor.motor_rotations_at_rpm_from_message(data=motor_info)
                                elif queue_data[0] in ['smmp']:
                                    job_ok = motor.motor_pulses_from_message(data=motor_info)
                                elif queue_data[0] in ['smmr']:
                                    job_ok = motor.motor_rotations_from_message(data=motor_info)
                                else:
                                    job_ok = -1

                                if job_ok == -1:
                                    rospy.loginfo(f"error assigning job {queue_data.message_id} to motor id: {motor.motor_id}")
                                else:
                                    motor_running = True
                                    timeout_flag = False
                                    motor_time = time.time()
                                    publish_progression_message(message_id=queue_data[1].message_id, status=3, motor_id=motor.motor_id)

                                    while motor_running:
                                        if motor.pulses_remaining == 0 and not motor.running and motor.status_update_count == 0:
                                            if motor.job_status == 0:
                                                motor_running = False
                                            elif timeout_flag:
                                                motor.job_complete(motor.job_id)
                                                motor_running = False

                                        if time.time() - start_time > required_job_time + 20:
                                            rospy.logerr(f"job id {queue_data[1].message_id} exceeded maximum time, motor(s) not complete")
                                            motor_running = False
                                            error_flag = True

                                        if time.time() - motor_time > expected_time_required[i] + 2:
                                            timeout_flag = True
                                            rospy.loginfo(f"timeout, {motor.pulses_remaining}, {motor.running}, {motor.status_update_count}")

                                        if time.time() - motor_time > expected_time_required[i] + 4:
                                            rospy.logerr(f"job id {queue_data[1].message_id} motor {motor.motor_id} exceeded maximum time")

                                            if motor.pulses_remaining != 0 and motor.running:
                                                motor.cancel_job()

                                            if not motor.sleep:
                                                motor.job_abort()

                                        rospy.sleep(0.05)

            else:
                error_flag = False
                pass

            if queue_data[0] in ['smmr', 'smmrar', 'smmpar', 'smmp', 'mmr', 'mmrar', 'mmpar', 'mmp']:
                if error_flag:
                    msg_status = 5
                    for motor in [motor_0, motor_1, motor_2, motor_3, motor_4, motor_5, motor_6, motor_7,
                                  motor_8, motor_9]:
                        if motor.pulses_remaining != 0 and motor.running:
                            motor.cancel_job()

                        if not motor.sleep:
                            motor.job_abort()

                    rospy.loginfo(f"job {queue_data[1].message_id} cancelled")

                else:
                    msg_status = 4
                    rospy.loginfo(f"job {queue_data[1].message_id} complete")

                publish_progression_message(message_id=queue_data[1].message_id, status=msg_status)
                rospy.sleep(1)

        rospy.sleep(0.05)

    SystemExit(0)
