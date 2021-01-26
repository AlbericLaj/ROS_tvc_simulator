#!/usr/bin/env python



import rospy
from tvc_simulator.msg import FSM
from tvc_simulator.msg import State
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import time


import rosbag

tStart = 5
tEnd = 200

position = np.zeros((1,3))
speed = np.zeros((1,3))
attitude = np.zeros((1,4))
omega = np.zeros((1,3))
prop_mass = np.zeros((1,1))
time_state = np.zeros((1,1))

control_force = np.zeros((1,3))
z_torque = np.zeros((1,1))
time_force = np.zeros((1,1))

bag = rosbag.Bag('../log/log.bag')
for topic, msg, t in bag.read_messages(topics=['/rocket_state']):
    new_pos = msg.pose.position
    new_speed = msg.twist.linear
    new_attitude = msg.pose.orientation    
    new_omega = msg.twist.angular
    new_mass = msg.propeller_mass
        
    position = np.append(position, [[new_pos.x, new_pos.y, new_pos.z]], axis = 0)
    speed = np.append(speed, [[new_speed.x, new_speed.y, new_speed.z]], axis = 0)
    attitude = np.append(attitude, [[ new_attitude.x, new_attitude.y, new_attitude.z, new_attitude.w]], axis = 0)
    omega = np.append(omega, [[new_omega.x, new_omega.y, new_omega.z]], axis = 0)
    prop_mass = np.append(prop_mass, [[new_mass]])
    time_state = np.append(time_state, [[t.to_sec()]])


for topic, msg, t in bag.read_messages(topics=['/control_pub']):
  new_force = msg.force
  control_force = np.append(control_force, [[new_force.x, new_force.y, new_force.z]], axis = 0)
  z_torque = np.append(z_torque, [[msg.torque.z]]) 
  time_force = np.append(time_force, [[t.to_sec()]]) 
   
bag.close()

prop_mass = prop_mass[1:]
speed = speed[1:]
omega = omega[1:]
position = position[1:]
attitude = attitude[1:]
time_state = time_state[1:]

control_force = control_force[1:]
z_torque = z_torque[1:]
time_force = time_force[1:]

time_force = time_force - time_state[0]
time_state = time_state - time_state[0]

r = R.from_quat(attitude)
attitude = r.as_euler('zyx', degrees=True)

omega = np.rad2deg(omega)



select = np.logical_and(time_state>tStart, time_state <tEnd)
select_force = np.logical_and(time_force>tStart, time_force <tEnd)

fig, axe = plt.subplots(3,3, figsize=(15,10))
l = axe[0][0].plot(time_state[select], position[:, 0:2][select])
axe[0][0].legend(l, ('X position [m]', 'Y position [m]'))


l = axe[0][1].plot(time_state[select], position[:, 2][select], label = 'Z position [m]')
axe[0][1].legend()

l = axe[1][0].plot(time_state[select], speed[:, 0:2][select])
axe[1][0].legend(l, ('X speed [m/s]', 'Y speed [m]'))

l = axe[1][1].plot(time_state[select], speed[:, 2][select],  label = 'Z speed [m/s]')
axe[1][1].legend()

l = axe[0][2].plot(time_state[select], attitude[select]) 
axe[0][2].legend(l, ('X [degree]', 'Y [degree]', 'Z [degree]'))

l = axe[1][2].plot(time_state[select], omega[select])
axe[1][2].legend(l, ('X speed [deg/s]', 'Y speed [deg/s]', 'Z speed [deg/s]'))

l = axe[2][2].plot(time_force[select_force], control_force[:, 0:2][select_force])
l = axe[2][2].plot(time_force[select_force], z_torque[select_force])
axe[2][2].legend(l, ('X force [N]', 'Y force [N]', 'Z torque [N.m]'))

l = axe[2][1].plot(time_force[select_force], control_force[:, 2][select_force], label = "Z force [N]")
axe[2][1].legend()

l = axe[2][0].plot(time_state[select], prop_mass[select], label = "propellant mass [kg]")
axe[2][0].legend()


plt.show()
