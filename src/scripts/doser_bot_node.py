#!/usr/bin/env python3

import warnings
import time
import threading
from pathlib import Path

# ROS Imports
import rospy

from doser_bot_package.msg import MotorStatus, MultiMotorRotationsAtRpmJob, MessageProgression

from src import parse_definitions_file, DoserBot, Motor

warnings.filterwarnings("ignore")

class MotorController:
    def __init__(self, definitions: dict, ppr: int = 200):

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

        # Publishers
        motor_status_publisher_id = rospy.get_param(param_name="~motor_status_msg_id", default='motor_status')
        message_progression_status_id = rospy.get_param(param_name="~message_progression_status_id", default='message_progression_status')
        self.motor_status_publisher = rospy.Publisher(motor_status_publisher_id, MotorStatus, queue_size=10)
        self.message_progression_status_publisher = rospy.Publisher(message_progression_status_id, MessageProgression, queue_size=1)

        # Subscribers(s)
        multi_motor_rotations_at_rpm_id = rospy.get_param(param_name="~multi_motor_rotations_at_rpm_id", default='/multi_motor_rotations_at_rpm')
        self.multi_motor_rotations_at_rpm_subscriber = rospy.Subscriber(multi_motor_rotations_at_rpm_id, MultiMotorRotationsAtRpmJob, self.multi_motor_rotations_at_rpm_callback, queue_size=1)

        self.doser_bot_messages_thread = None
        self.read_lock = threading.Lock()

        self.doser_bot_messages_thread = threading.Thread(target=self.update, daemon=True)
        self.doser_bot_messages_thread.start()


    def multi_motor_rotations_at_rpm_callback(data):

        if new_request.motor in doserbot.motor_ids_in_use:
            return doserbot.send_motor_rotations_at_set_rpm(motor_id=new_request.motor,
                                                            number_or_rotations=new_request.rotations,
                                                            rpm=new_request.rpm,
                                                            direction=False if new_request.direction == 0 else True,
                                                            use_ramping=False if new_request.use_ramping == 0 else True,
                                                            ramping_steps=new_request.ramping_steps,
                                                            job_id=new_request.job_id,
                                                            )

    def publish_motor_status_message():
        stamp = rospy.get_rostime()
        
        for i, motor_id in enumerate(self.doserbot.motor_ids_in_use):
            motor_message = MotorStatus()
            motor_message.header.stamp = stamp
            motor_message.motor = motor_id
            motor_message.status_bits = self.doserbot.motor_status_dict[f'motor_{motor_id}_status']
            motor_message.job_id = self.doserbot.motor_status_dict[f'motor_{motor_id}_job_id']
            motor_message.pulses_remaining = self.doserbot.motor_status_dict[f'motor_{motor_id}_pulses_remaining']
            self.motor_status_publisher.publish(motor_message)


    def update(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for response in self.doserbot.get_response_messages():
                pass

            for job in self.doserbot.get_job_complete_messages():
                pass

            for job in self.doserbot.get_job_cancelled_messages():
                pass

            self.publish_motor_status_message()

            rate.sleep()



def shutdown_hook():
    rospy.loginfo(f"doser_bot_node shutdown")


if __name__ == "__main__":
    desc = """Insert meaningful description here"""

    PROJECT_DIR = Path(__file__).resolve().parents[1]
    header_file = PROJECT_DIR / 'doser_bot_arduino/definitions.h'

    definitions_dict = parse_definitions_file(header_file)



    # Create a ROS node
    rospy.init_node("doser_bot_node")
    rospy.on_shutdown(shutdown_hook)



    time.sleep(1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():


        rate.sleep()

    doserbot.stop()

SystemExit(0)
