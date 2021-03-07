#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/DroneState.h"

#include "tvc_simulator/DroneControl.h"

#include "tvc_simulator/GetFSM.h"

#define SERVO_NUMBER 1

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "actuators_test");
    ros::NodeHandle n;

    // Create control publisher
    ros::Publisher control_pub = n.advertise<tvc_simulator::DroneControl>("drone_control_pub", 10);

    // Setup Time_keeper client and srv variable for FSM and time synchronization
    ros::ServiceClient client_fsm = n.serviceClient<tvc_simulator::GetFSM>("getFSM");
    tvc_simulator::GetFSM srv_fsm;

    // Initialize control
    tvc_simulator::DroneControl control_law;

    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = n.createTimer(ros::Duration(0.05), [&](const ros::TimerEvent &) {

        // Get current FSM and time
        if (client_fsm.call(srv_fsm)) {
            current_fsm = srv_fsm.response.fsm;
        }

        // State machine ------------------------------------------
        if (current_fsm.state_machine.compare("Idle") == 0) {
            // Do nothing
        } else if (current_fsm.state_machine.compare("Launch") == 0) {
            float servo_angle;
            const float period = 3.0;
            if((int)(current_fsm.time_now/period) % 2 == 0){
                servo_angle = 30*M_PI/180;
            }
            else{
                servo_angle = -30*M_PI/180;
            }

            control_law.servo1 = 0;
            control_law.servo2 = 0;
            control_law.top = 20;
            control_law.bottom = 20;

            if(SERVO_NUMBER == 1)control_law.servo1 = servo_angle;
            else control_law.servo2 = servo_angle;

        } else if (current_fsm.state_machine.compare("Coast") == 0) {

            control_law.servo1 = 0;
            control_law.servo2 = 0;
            control_law.top = 0;
            control_law.bottom = 0;
        }

        control_pub.publish(control_law);
    });

    // Automatic callback of service and publisher from here
    ros::spin();

}
