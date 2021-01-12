#!/usr/bin/env python3

import rospy

import numpy as np
import math

from tvc_simulator.msg import Control
from tvc_simulator.msg import FSM
from tvc_simulator.msg import State

# Create global control variables
thrust_force = np.array([0, 0, 0])
thrust_torque = np.array([0, 0, 0])

# Create global variable fsm
current_fsm = FSM()

# Create global variable rocket_state
rocket_state = State()


def fsm_callback(fsm):
	current_fsm.time_now = fsm.time_now
	current_fsm.state_machine = fsm.state_machine


def rocket_state_callback(state):
	rocket_state = state

class Rocket:
  dry_mass = 0
  propellant_mass = 0
  dry_CM = 0

  Isp = 0
  maxThrust = 0
  minThrust = 0

  ground_altitude = 0
  groung_temperature = 0
  

  target_apogee = np.zeros(3);
  Cd = np.zeros(3)
  surface = np.zeros(3)
  length = 0
  diameter = np.zeros(3)

  def __init__(self):
	  rocket_data = rospy.get_param("/rocket")
	  env_data = rospy.get_param("/environment")
    
	  self.dry_mass = rocket_data["dry_mass"]
	  self.propellant_mass = rocket_data["propellant_mass"]
	  self.dry_CM = rocket_data["dry_CM"]

	  self.Isp = rocket_data["Isp"]
	  self.maxThrust = rocket_data["maxThrust"]
	  self.minThrust = rocket_data["minThrust"]

	  self.ground_altitude = env_data["ground_altitude"]
	  self.ground_temperature = env_data["temperature"]

	  nStage = rocket_data["stages"]
	  self.Cd = np.asarray(rocket_data["Cd"])
	  self.diameter = np.asarray(rocket_data["diameters"])
	  self.length = rocket_data["stage_z"][nStage-1]


	  self.surface[0] = self.diameter[1]*self.length;
	  self.surface[1] = self.surface[0];
	  self.surface[2] = self.diameter[1]*self.diameter[1]/4 * 3.14159;
    
    
  def getPression(self, z):
    return 101325*np.exp(-0.00012*(z+ self.ground_altitude))

  def getDensity(self, z):
    return self.getPression(z)/(287.058*self.ground_temperature)


class Disturbance:
  wind_gust_intensity = np.zeros(2)
  wind_gust_assymetry = 0
  wind_gust_var = 0

  motor_tilt = np.zeros(2)
  plume_tilt_var = np.zeros(2)

  fins_tilt = np.zeros(3)
  drag_assymetry = np.zeros(2)

  air_density_bias = 0


  def __init__(self):
	  chaos_data = rospy.get_param("/perturbation")

	  self.wind_gust_intensity = np.asarray(chaos_data["wind_gust_intensity"])
	  self.wind_gust_assymetry = chaos_data["wind_gust_assymetry"]
	  self.wind_gust_var = chaos_data["wind_gust_var"]

	  self.motor_tilt = np.asarray(chaos_data["motor_tilt"])
	  self.plume_tilt_var = np.asarray(chaos_data["plume_tilt_var"])

	  self.fins_tilt = np.asarray(chaos_data["fins_tilt"])
	  self.drag_assymetry = np.asarray(chaos_data["drag_assymetry"])
	  
	  self.air_density_bias = chaos_data["air_density_bias"]


  def get_gust_disturbance(self, rocket, state):
    wind_speed = np.random.normal(self.wind_gust_intensity, self.wind_gust_var*self.wind_gust_intensity/100) 

    rho_air = rocket.getDensity(state.pose.position.z)
    force = 0.5*rocket.Cd[0:2]*rocket.surface[0:2]*rho_air*wind_speed**2 

    gust_position = (2*np.random.rand()-1)*self.wind_gust_assymetry*rocket.length/2
    torque = force*gust_position
    
    disturbance = Control()
    disturbance.torque.x = torque[0]
    disturbance.torque.y = torque[1]
    disturbance.torque.z = 0

    disturbance.force.x = force[0]
    disturbance.force.y = force[1]
    disturbance.force.z = 0

  
    return disturbance








if __name__ == '__main__':

	rospy.init_node('disturbance', anonymous=True)

	# Init global variable
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

	# Subscribe to fsm 
	rospy.Subscriber("fsm_pub", FSM, fsm_callback)

	# Subscribe to rocket_state 
	rospy.Subscriber("rocket_state", State, rocket_state_callback)

	# Publisher for disturbance control
	disturbance_pub = rospy.Publisher('disturbance_pub', Control, queue_size=10)

	rocket = Rocket()
	chaos = Disturbance()

	current_disturbance = chaos.get_gust_disturbance(rocket, rocket_state)

  # Disturbance rate in Hz
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():

		# Thread sleep time defined by integration_period variable
		rate.sleep()

		current_disturbance = chaos.get_gust_disturbance(rocket, rocket_state)

		disturbance_pub.publish(current_disturbance)
    
		
			 



