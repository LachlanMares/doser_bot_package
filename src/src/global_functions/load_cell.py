#!/usr/bin/env python3

import rospy

from doser_bot_package.msg import LoadCellStatus


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