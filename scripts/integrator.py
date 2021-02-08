#!/usr/bin/env python3

# -----------------------
#
# Simulation node: Integrate rocket equations to provide real time rocket state for other nodes
#
# Input:
#    - Rocket control law (3D force and 3D torque) from control node
#		 - External disturbances (3D force and 3D torque) from disturbance node --> TBD
#
# Parameters:
#    - Rocket model defined in JSON file --> TBD
#    - Environment model defined in JSON file --> TBD
#    - Integration time defined by integration_period variable
#    - Initial state (position, speed, quaternion, angular speed)
#
# Outputs:
#    - Rocket state (position, speed, quaternion, angular speed)
#
# -----------------------

import rospy
from tvc_simulator.msg import FSM
from tvc_simulator.msg import State
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor

import numpy as np
import math
from scipy.integrate import ode, solve_ivp
#import matplotlib.pyplot as plt
import time
import yaml

from Rocket.Body import Body
from Rocket.Fins import Fins
from Rocket.Motor import Motor
from Rocket.Rocket import Rocket
from Rocket.Stage import Stage
from Functions import Math
from Functions.Math.quat2rotmat import quat2rotmat
from Functions.Math.rot2anglemat import rot2anglemat
from Functions.Math.normalize_vector import normalize_vector
from Functions.Math.quat_evolve import quat_evolve
from Functions.Models.wind_model import wind_model
from Functions.Models.robert_galejs_lift import robert_galejs_lift
from Functions.Models.barrowman_lift import barrowman_lift
from Functions.Math.rot2quat import rot2quat
from Functions.Models.stdAtmosUS import stdAtmosUS
from Simulator3D import Simulator3D
from Functions.Models.stdAtmos import stdAtmos



def control_callback(control):
	global current_control
	current_control = control

def disturbance_callback(chaos):
	global current_disturbance
	current_disturbance = chaos

def fsm_callback(fsm):
	global current_fsm
	current_fsm.time_now = fsm.time_now
	current_fsm.state_machine = fsm.state_machine

def init_integrator():
	# Get ROS parameters from YAML file 
	
	rocket_data = rospy.get_param("/rocket")
	env_data = rospy.get_param("/environment")

  # Rocket definition
	position_cone = rocket_data["stage_z"][1]

	cone = Body('tangent ogive', rocket_data["diameters"][0:2], rocket_data["stage_z"][0:2])

	tube = Body("cylinder", rocket_data["diameters"][2:],
                        np.array(rocket_data["stage_z"][2:])-position_cone)

	rocket_cone = Stage('Bellalui 2 nosecone', cone, 0, 0, np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]))

	rocket_body = Stage('Bellalui 2 body', tube, rocket_data["dry_mass"], rocket_data["dry_CM"], np.diag(rocket_data["dry_I"]))

	finDefData = {'number': rocket_data["fin_n"],
                'root_chord': rocket_data["fin_root_chord"],
                'tip_chord': rocket_data["fin_tip_chord"],
                'span': rocket_data["fin_span"],
                'sweep': rocket_data["fin_sweep"],
                'thickness': rocket_data["fin_thickness"],
                'phase': 0,
                'body_top_offset': rocket_data["fin_top_offset"],
                'total_mass': 0}

	rocket_body.add_fins(finDefData)

	rocket_body.add_motor('Motors/M2400T.txt')
  
	main_parachute_params = [True, 23.14, 100]
	rocket_body.add_parachute(main_parachute_params)

	drogue_parachute_params = [False, 1.75, 1000]
	rocket_body.add_parachute(drogue_parachute_params)

	Bellalui_2 = Rocket()
  
	Bellalui_2.add_stage(rocket_cone)
	Bellalui_2.add_stage(rocket_body)
  
	Bellalui_2.set_propellant_mass(rocket_data["propellant_mass"])
	Bellalui_2.set_propellant_CG(rocket_data["propellant_CM"])

	Bellalui_2.add_cg_empty_rocket(rocket_data["dry_CM"])
	Bellalui_2.set_rocket_inertia(np.diag(rocket_data["dry_I"]))
	Bellalui_2.set_payload_mass(0)

	Bellalui_2.set_motor_Isp(rocket_data["Isp"])
  
	US_Atmos = stdAtmosUS(1567, 290.15, 84972.484, 0.51031)
  
	SimObj = Simulator3D(Bellalui_2, US_Atmos)
	return SimObj
	

if __name__ == '__main__':

	# Creates global variables
	current_disturbance = Control()

	current_control = Control()

	current_fsm = FSM()

	# Init ROS
	rospy.init_node('integrator', anonymous=True)

	# Init global variable
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

	# Subscribe to control law
	rospy.Subscriber("control_pub", Control, control_callback)

	# Subscribe to fsm 
	rospy.Subscriber("fsm_pub", FSM, fsm_callback)

	# Subscribe to disturbance 
	rospy.Subscriber("disturbance_pub", Control, disturbance_callback)

	# Publisher for rocket state
	rocket_state_pub = rospy.Publisher('rocket_state', State, queue_size=10)

	# Publisher for fake rocket sensor data
	sensor_pub = rospy.Publisher('sensor_pub', Sensor, queue_size=10)

	# Time step between integration in [s], used for thread frequency and integration time
	integration_period = 0.10 
	rate = rospy.Rate(1/integration_period) # in Hz

	# Simulation object
	rocket_sim = init_integrator()

	# Init state (position, speed, quaternion, angular speed)
	S_new = np.array([0,0,0, 0,0,30,     0.0, 0.0,  0.0,  1.0,      0.0, 0.0, 0.0     , rocket_sim.rocket.get_propellant_mass()])
	T_new = 0
	
	solver_dopri5 =  ode(rocket_sim.Dynamics_6DOF).set_integrator('dopri5') 

	while not rospy.is_shutdown():

		# Thread sleep time defined by integration_period variable
		rate.sleep()
		
		# Start simulation only on "launch" mode
		if current_fsm.state_machine == "Idle":
			S_new = S_new

		else:
			thrust_force = np.zeros(3)
			thrust_torque = np.zeros(3)

      # Force and torque is sum of control and disturbance
			if current_fsm.state_machine == "Launch":

				thrust_force[0] = current_control.force.x# + current_disturbance.force.x
				thrust_force[1] = current_control.force.y# + current_disturbance.force.y
				thrust_force[2] = current_control.force.z# + current_disturbance.force.z

				thrust_torque[0] = current_control.torque.x# + current_disturbance.torque.x
				thrust_torque[1] = current_control.torque.y# + current_disturbance.torque.y
				thrust_torque[2] = current_control.torque.z# + current_disturbance.torque.z

      # Force and torque is only disturbance (no more fuel for control)
			elif current_fsm.state_machine == "Coast":

				thrust_force[0] = current_disturbance.force.x
				thrust_force[1] = current_disturbance.force.y
				thrust_force[2] = current_disturbance.force.z

				thrust_torque[0] = current_disturbance.torque.x
				thrust_torque[1] = current_disturbance.torque.y
				thrust_torque[2] = current_disturbance.torque.z
				
				thrust_force = np.zeros(3)
				thrust_torque = np.zeros(3)
		  
      # Now do the integrationwith computed force and torque 
			start_time = rospy.get_time()
			
			# Actual integration of the state "S_new" using the control law
			integration_ivp = solve_ivp(rocket_sim.Dynamics_6DOF, [T_new, T_new+integration_period], S_new, method = 'RK23', args = (thrust_force, thrust_torque))
			
			#solver_dopri5.set_initial_value(S_new, T_new).set_f_params(thrust_force, thrust_torque)
			#S_new = solver_dopri5.integrate(T_new+integration_period)
			#T_new = T_new + integration_period

			# Get final state and time to be used for next iteration
			S_new = integration_ivp.y[:, -1]
			T_new = integration_ivp.t[-1]

			# Used to time integration process
			#rospy.loginfo(1000*(rospy.get_time()-start_time))
		

 
      
		# Parse state and publish it on the /rocket_state topic
		rocket_state = State()

		rocket_state.pose.position.x = S_new[0]
		rocket_state.pose.position.y = S_new[1]
		rocket_state.pose.position.z = S_new[2]

		rocket_state.twist.linear.x = S_new[3]
		rocket_state.twist.linear.y = S_new[4]
		rocket_state.twist.linear.z = S_new[5]

		rocket_state.pose.orientation.x = S_new[6]
		rocket_state.pose.orientation.y = S_new[7]
		rocket_state.pose.orientation.z = S_new[8]
		rocket_state.pose.orientation.w = S_new[9]

		rocket_state.twist.angular.x = S_new[10]
		rocket_state.twist.angular.y = S_new[11]
		rocket_state.twist.angular.z = S_new[12]

		rocket_state.propeller_mass = S_new[13]
		
		rocket_state_pub.publish(rocket_state)

		# Parse sensor data and publish it on /sensor_pub topic
		current_sensor = Sensor()
		
		[IMU_acc, IMU_gyro, baro_height] = rocket_sim.rocket.get_sensor_data()

		current_sensor.IMU_acc.x = IMU_acc[0]
		current_sensor.IMU_acc.y = IMU_acc[1]
		current_sensor.IMU_acc.z = IMU_acc[2]

		current_sensor.IMU_gyro.x = IMU_gyro[0]
		current_sensor.IMU_gyro.y = IMU_gyro[1]
		current_sensor.IMU_gyro.z = IMU_gyro[2]

		current_sensor.baro_height = baro_height

		sensor_pub.publish(current_sensor)

