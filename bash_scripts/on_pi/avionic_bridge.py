#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import pigpio 
import time

from drone_tvc_simulator.msg import DroneControl
from drone_tvc_simulator.msg import FSM
from drone_tvc_simulator.msg import DroneState

def control_callback(control):
    global current_control
    current_control = control

    top_motor_cmd = control.thrust + control.torque
    bottom_motor_cmd = control.thrust - control.torque

    # saturate inputs
    control.servo1 = min(max(control.servo1, -0.8), 0.8)
    control.servo1 = min(max(control.servo1, -0.8), 0.8)

    # saturate at 60% //TODO increase
    top_motor_cmd = min(max(top_motor_cmd, 0), 60)
    bottom_motor_cmd = min(max(bottom_motor_cmd, 0), 60)


    # convert to PWM
    yaw_DC = ((control.servo1/np.pi) + 1.45)*1000
    pitch_DC = ((control.servo2/np.pi) + 1.55)*1000

    top_motor_DC = top_motor_cmd*10 + 1000
    bottom_motor_DC = bottom_motor_cmd*10 + 1000

    # send to motors
    pi.set_servo_pulsewidth(pitch_pin, pitch_DC) # update position
    pi.set_servo_pulsewidth(yaw_pin, yaw_DC) # update position

    pi.set_servo_pulsewidth(top_motor_pin, top_motor_DC)
    pi.set_servo_pulsewidth(bottom_motor_pin, bottom_motor_DC)

if __name__ == '__main__':

    # Create global variable
    rocket_state = State()
    rocket_state.propeller_mass = 0.7
    
    current_control = Control()

    # Init ROS
    rospy.init_node('avionic_bridge', anonymous=True)
    
    # Subscribe to rocket_control 
    rospy.Subscriber("control_pub", Control, control_callback)
    
    # Publisher for rocket state from AV control
    rocket_state_pub = rospy.Publisher('rocket_state', State, queue_size=10)
    
    # Config Rpi
    ser = serial.Serial('/dev/serial0', 115200)  # open serial port
    
    yaw_pin = 23
    pitch_pin = 24
    top_motor_pin = 8
    bottom_motor_pin = 25

    pi = pigpio.pi()

    pi.set_servo_pulsewidth(top_motor_pin, 0)
    time.sleep(1)
    pi.set_servo_pulsewidth(top_motor_pin, 2000)
    time.sleep(1)
    pi.set_servo_pulsewidth(top_motor_pin, 1000)
    time.sleep(1)

    pi.set_servo_pulsewidth(bottom_motor_pin, 0)
    time.sleep(1)
    pi.set_servo_pulsewidth(bottom_motor_pin, 2000)
    time.sleep(1)
    pi.set_servo_pulsewidth(bottom_motor_pin, 1000)
    time.sleep(1)

    pi.set_servo_pulsewidth(yaw_pin, 1500)
    pi.set_servo_pulsewidth(pitch_pin, 1500)

    pi.set_servo_pulsewidth(top_motor_pin, 1000)
    pi.set_servo_pulsewidth(bottom_motor_pin, 1000)
 

    
    # Node rate in Hz
    rate = rospy.Rate(200)
    n_average = 100
    baro = np.zeros(n_average)

    while not rospy.is_shutdown():
    
        # Thread sleep time defined by rate
        rate.sleep()
        
        #-----------------------------------------------------------
        
        line = ser.readline()
        if line[0] == "S" and line[1] == "S" : # SS symbol = sensor message
            # Decode byte array
            line = line[2:-2]
            line_ascii = line.decode('ascii')
            line_ascii = line_ascii.split(',')
        
            rocket_state_raw = np.array(line_ascii, dtype = np.float)
           # print(rocket_state_raw)
            
            # Parse state and publish it on the /drone_state topic
	    baro = np.append(baro[1:],rocket_state_raw[7]/100)
            new_baro = np.convolve(baro, np.ones(n_average)/n_average, mode='valid')
 
            rocket_state.pose.position.x = rocket_state_raw[5]/100
            rocket_state.pose.position.y = rocket_state_raw[6]/100
            rocket_state.pose.position.z = new_baro

            rocket_state.twist.linear.x = 0
            rocket_state.twist.linear.y = 0
            rocket_state.twist.linear.z = 0

            rocket_state.pose.orientation.x = rocket_state_raw[2]
            rocket_state.pose.orientation.y = rocket_state_raw[3]
            rocket_state.pose.orientation.z = rocket_state_raw[4]
            rocket_state.pose.orientation.w = rocket_state_raw[1]

            rocket_state.twist.angular.x = 0
            rocket_state.twist.angular.y = 0
            rocket_state.twist.angular.z = 0

            
        rocket_state_pub.publish(rocket_state)
        
    
