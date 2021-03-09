#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import pigpio 
import time

from tvc_simulator.msg import DroneControl
from tvc_simulator.msg import FSM
from tvc_simulator.msg import DroneState
from tvc_simulator.srv import GetFSM

SERVO_NUMBER = 1

if __name__ == '__main__':

    current_control = DroneControl()

    # Init ROS
    rospy.init_node('actuators_test', anonymous=True)

    drone_control_pub = rospy.Publisher('drone_control_pub', DroneControl, queue_size=10)

    client_fsm = rospy.ServiceProxy('getFSM', GetFSM)

    control_law = DroneControl()
    control_law.servo1 =0 
    control_law.servo2 = 0
    control_law.top = 0
    control_law.bottom = 0
    
    # Node rate in Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        resp = client_fsm()
        current_fsm = resp.fsm
        rospy.loginfo(current_fsm)
        if current_fsm.state_machine == "Idle":
            pass
        elif current_fsm.state_machine == "Launch":
            servo_angle = 0
            period = 3
            if round(current_fsm.time_now/period) % 2 == 0:
                servo_angle = 30*np.pi/180;
            else:
                servo_angle = -30*np.pi/180;
            control_law.servo1 = 0
            control_law.servo2 = 0
            control_law.top = 0
            control_law.bottom = 0
            if SERVO_NUMBER == 1:
                control_law.servo1 = servo_angle
            else:
                control_law.servo2 = servo_angle


        elif current_fsm.state_machine == "Coast":
            pass
        drone_control_pub.publish(control_law)
        
    
