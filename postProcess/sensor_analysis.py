#!/usr/bin/env python

import rospy
from tvc_simulator.msg import State
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import csv

import time
import rosbag

time_end = 5

# -------------- Read simu data ---------------------
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

for topic, msg, t in bag.read_messages(topics=['/control_pub']):
  if(msg.force.z):
    time_init = t.to_sec()
    break

bag.close()

time_simu = np.array(time_simu)-time_init
accelerometer = np.array(accelerometer)
gyroscope = np.array(gyroscope)
barometer = np.array(barometer)

select_simu = np.logical_and(time_simu>=0, time_simu <time_end)


# -------------- Read flight data ---------------------
raw_data = []

with open('data_Wasserfallen.csv', "r") as csvfile:
  line = csv.reader(csvfile, delimiter=',', skipinitialspace=True)
  for row in line:
    raw_data.append(row)

raw_data = np.array(raw_data)

raw_data = np.reshape(raw_data, (-1,11))
raw_data = raw_data[1:,1:]

raw_data = raw_data.astype(np.float)

time_data = raw_data[:, 0] - 1849.35
select_data = np.logical_and(time_data>=0, time_data <time_end)


time_data = time_data[select_data]

accelerometer_data = raw_data[:, 4:7][select_data]*9.81
gyroscope_data = raw_data[:, 7:][select_data]
baro_data = raw_data[:,2][select_data]

print("Apogee: {}".format(max(barometer)))


# Plot data -----------------------------------------------

fig, axe = plt.subplots(2,3, figsize=(15,10))

l = axe[0][0].plot(time_simu[select_simu], accelerometer[select_simu])
axe[0][0].legend(l, ('X acc [m/s^2]', 'Y acc [m/s^2]', 'Z acc [m/s^2]'))

l = axe[0][1].plot(time_simu[select_simu], gyroscope[select_simu])
axe[0][1].legend(l, ('X gyro [rad/s]', 'Y gyro [rad/s]', 'Z gyro [rad/s]'))

l = axe[0][2].plot(time_simu[select_simu], barometer[select_simu])
axe[0][2].legend(l, ('barometer [m]'))

l = axe[1][0].plot(time_data, accelerometer_data)
axe[0][0].plot(time_data, accelerometer_data, "+")
axe[1][0].legend(l, ('X acc [m/s^2]', 'Y acc [m/s^2]', 'Z acc [m/s^2]'))

l = axe[1][1].plot(time_data, gyroscope_data)
axe[1][1].legend(l, ('X gyro [rad/s]', 'Y gyro [rad/s]', 'Z gyro [rad/s]'))

l = axe[1][2].plot(time_data, baro_data)
axe[1][2].legend(l, ('barometer [m]'))



plt.show()