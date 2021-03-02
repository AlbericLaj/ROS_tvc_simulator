#!/usr/bin/env python

import rospy
from tvc_simulator.msg import State
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import time


import rosbag

bag = rosbag.Bag('../log/log.bag')

time_simu = []
accelerometer = []
gyroscope = []
barometer = []

for topic, msg, t in bag.read_messages(topics=['/sensor_pub']):
  time_simu.append(t.to_sec())
  accelerometer.append(np.array([msg.IMU_acc.x, msg.IMU_acc.y, msg.IMU_acc.z]))
  gyroscope.append(np.array([msg.IMU_gyro.x, msg.IMU_gyro.y, msg.IMU_gyro.z]))
  barometer.append(msg.baro_height)

bag.close()


accelerometer = np.array(accelerometer)
gyroscope = np.array(gyroscope)
barometer = np.array(barometer)

fig, axe = plt.subplots(1,3, figsize=(15,10))

l = axe[0].plot(time_simu, accelerometer)
axe[0].legend(l, ('X acc [m/s^2]', 'Y acc [m/s^2]', 'Z acc [m/s^2]'))

l = axe[1].plot(time_simu, gyroscope)
axe[1].legend(l, ('X gyro [rad/s]', 'Y gyro [rad/s]', 'Z gyro [rad/s]'))

l = axe[2].plot(time_simu, gyroscope)
axe[2].legend(l, ('barometer [m]'))

plt.show()