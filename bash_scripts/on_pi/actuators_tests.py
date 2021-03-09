#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import time

from tvc_simulator.msg import DroneControl
from tvc_simulator.msg import FSM
from tvc_simulator.msg import DroneState
from std_msgs.msg import String
from tvc_simulator.srv import GetFSM

# define the test sequence
PERIODIC_SEQUENCE = False


SERVO1_SEQUENCE = []
SERVO2_SEQUENCE = []

TOP_SEQUENCE = range(0, 61, 10)
BOTTOM_SEQUENCE = range(0, 61, 10)

TIME_SEQUENCE = len(TOP_SEQUENCE)*[3]


TOP_SEQUENCE = [30]
BOTTOM_SEQUENCE = [30]
TIME_SEQUENCE = [3]

previous_state = DroneState()
current_state = DroneState()

def stateCallback(state):
    global current_state, previous_state
    previous_state = current_state
    current_state = state

if __name__ == '__main__':

    current_control = DroneControl()

    # Init ROS
    rospy.init_node('actuators_test', anonymous=True)

    drone_control_pub = rospy.Publisher('drone_control_pub', DroneControl, queue_size=10)

    rospy.Subscriber("drone_state", DroneState, stateCallback)

    rospy.wait_for_service('getFSM')
    client_fsm = rospy.ServiceProxy('getFSM', GetFSM)

    coast_pub = rospy.Publisher('commands', String, queue_size=10)

    control_law = DroneControl()
    control_law.servo1 = 0
    control_law.servo2 = 0
    control_law.top = 0
    control_law.bottom = 0

    # Node rate in Hz
    rate = rospy.Rate(40)

    seq_time_start = None
    seq_idx = 0

    while not rospy.is_shutdown():
        resp = client_fsm()
        current_fsm = resp.fsm
        # rospy.loginfo(current_fsm)
        if current_fsm.state_machine == "Idle":
            pass
        elif current_fsm.state_machine == "Launch":
            if seq_time_start is None:
                seq_time_start = current_fsm.time_now
            if current_fsm.time_now - seq_time_start > TIME_SEQUENCE[seq_idx]:
                seq_idx += 1
                seq_time_start = current_fsm.time_now
                if seq_idx >= len(TIME_SEQUENCE):
                    if PERIODIC_SEQUENCE:
                        seq_idx = 0
                    else:
                        coast_command = String("Coast")
                        coast_pub.publish(coast_command)
                        continue
            control_law.servo1 = (SERVO1_SEQUENCE[seq_idx] if seq_idx < len(SERVO1_SEQUENCE) else 0) * np.pi / 180
            control_law.servo2 = (SERVO2_SEQUENCE[seq_idx] if seq_idx < len(SERVO2_SEQUENCE) else 0) * np.pi / 180
            control_law.top = (TOP_SEQUENCE[seq_idx] if seq_idx < len(TOP_SEQUENCE) else 0)
            control_law.bottom = (BOTTOM_SEQUENCE[seq_idx] if seq_idx < len(BOTTOM_SEQUENCE) else 0)

            kp = 1
            # kd = kp/10
            kd = 0
            pd_control = -current_state.pose.orientation.z * kp - (current_state.pose.orientation.z - previous_state.pose.orientation.y) * kd
            # TODO check orientation
            control_law.top += pd_control
            control_law.bottom -= pd_control

        elif current_fsm.state_machine == "Coast":
            control_law.servo1 = 0
            control_law.servo2 = 0
            control_law.top = 0
            control_law.bottom = 0
            pass
        drone_control_pub.publish(control_law)
