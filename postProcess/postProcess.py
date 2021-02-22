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

tStart = -1
tEnd = 7

position = np.zeros((1,3))
speed = np.zeros((1,3))
attitude = np.zeros((1,4))
omega = np.zeros((1,3))
prop_mass = np.zeros((1,1))
time_state = np.zeros((1,1))

control_force = np.zeros((1,3))
z_torque = np.zeros((1,1))
time_force = np.zeros((1,1))

target_position = np.zeros((1,3))
target_speed = np.zeros((1,3))
target_prop_mass = np.zeros((1,1))
time_target = np.zeros((1,1))

bag = rosbag.Bag('../log/log.bag')

for topic, msg, t in bag.read_messages(topics=['/fsm_pub']):
  time_init = t.to_sec()
  break
  

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
  
for topic, msg, t in bag.read_messages(topics=['/waypoint_pub']):
  new_target_pos = msg.position
  new_target_speed = msg.speed
  
  target_position = np.append(target_position, [[new_target_pos.x, new_target_pos.y, new_target_pos.z]], axis = 0)
  target_speed = np.append(target_speed, [[new_target_speed.x, new_target_speed.y, new_target_speed.z]], axis = 0)
  target_prop_mass = np.append(target_prop_mass, [[msg.propeller_mass]])
  time_target = np.append(time_target, [[msg.time]])
   
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

target_position = target_position[1:]
target_speed = target_speed[1:]
target_prop_mass = target_prop_mass[1:]
time_target = time_target[1:]


time_force = time_force - time_init
time_state = time_state - time_init

quaternion = attitude
r = R.from_quat(attitude)
attitude = r.as_euler('zyx', degrees=True)

omega = np.rad2deg(omega)


select = np.logical_and(time_state>tStart, time_state <tEnd)
select_force = np.logical_and(time_force>tStart, time_force <tEnd)

target_positionZ = target_position[:, 2]
target_speedZ = target_speed[:, 2]

target_positionZ = np.reshape(target_positionZ, (-1, 10))
target_speedZ = np.reshape(target_speedZ, (-1, 10))
target_prop_mass = np.reshape(target_prop_mass, (-1, 10))
time_target = np.reshape(time_target, (-1, 10))


select_target = np.logical_and(time_target>tStart, time_target <tEnd)

time_target = time_target[select_target]
target_prop_mass = target_prop_mass[select_target]
target_positionZ = target_positionZ[select_target]
target_speedZ = target_speedZ[select_target]



fig, axe = plt.subplots(3,4, figsize=(15,10))


l = axe[0][0].plot(time_state[select], position[:, 0:2][select])
axe[0][0].legend(l, ('X position [m]', 'Y position [m]'))


l = axe[0][1].plot(time_state[select], position[:, 2][select], label = 'Z position [m]', linewidth=4)
#l = axe[0][1].plot(time_target.T, target_positionZ.T)
axe[0][1].legend()

l = axe[1][0].plot(time_state[select], speed[:, 0:2][select])
axe[1][0].legend(l, ('X speed [m/s]', 'Y speed [m]'))

l = axe[1][1].plot(time_state[select], speed[:, 2][select],  label = 'Z speed [m/s]')
#l = axe[1][1].plot(time_target.T, target_speedZ.T)
axe[1][1].legend()

l = axe[0][2].plot(time_state[select], attitude[:, 1:3][select]) 
#axe[0][2].axhline(y=-180, color='r', linestyle='--')
#axe[0][2].axhline(y=180, color='r', linestyle='--')
axe[0][2].legend(l, ('X [degree]', 'Y [degree]'))

#l = axe[0][3].plot(time_state[select], quaternion[select])
#axe[0][3].legend(l, ('W','X', 'Y', 'Z'))
l = axe[0][3].plot(time_state[select], attitude[:,0][select],  label = 'Z [degree]', color = "green")
#axe[0][3].axhline(y=-180, color='r', linestyle='--')
#axe[0][3].axhline(y=180, color='r', linestyle='--')
axe[0][3].legend()

l = axe[1][2].plot(time_state[select], omega[:, 0:2][select])
axe[1][2].legend(l, ('X speed [deg/s]', 'Y speed [deg/s]'))

l = axe[1][3].plot(time_state[select], omega[:, 2][select],  label = 'Z speed [deg/s]', color = "green")
axe[1][3].legend()

l = axe[2][2].plot(time_force[select_force], control_force[:, 0:2][select_force])
axe[2][2].legend(l, ('X force [N]', 'Y force [N]'))

l = axe[2][3].plot(time_force[select_force], z_torque[select_force], label = 'Z torque [N.m]', color = "green")
axe[2][3].legend()

l = axe[2][1].plot(time_force[select_force], control_force[:, 2][select_force], label = "Z force [N]")
axe[2][1].legend()

l = axe[2][0].plot(time_state[select], prop_mass[select], label = "propellant mass [kg]")
l = axe[2][0].plot(time_target.T, target_prop_mass.T)
axe[2][0].legend()

fig.tight_layout()


plt.show()

