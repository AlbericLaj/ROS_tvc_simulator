#ifndef SRC_DRONE_MODEL_H
#define SRC_DRONE_MODEL_H

#include "rocket_model.h"

class Drone {
public:
    float dry_mass;

    float maxTorque;
    float maxServo1Angle;
    float maxServo2Angle;

    float maxThrust;
    float minThrust;
    float CM_to_thrust_distance;

    std::vector<float> I{0, 0, 0};
    std::vector<float> J_inv{0, 0, 0};

    void init(ros::NodeHandle n) {
        n.getParam("/rocket/maxTorque", maxTorque);
        n.getParam("/rocket/maxThrust", maxThrust);
        n.getParam("/rocket/minThrust", minThrust);
        n.getParam("/rocket/maxServo1Angle", maxServo1Angle);
        n.getParam("/rocket/maxServo2Angle", maxServo2Angle);

        n.getParam("/rocket/dry_mass", dry_mass);

        n.getParam("/rocket/CM_to_thrust_distance", CM_to_thrust_distance);
        n.getParam("/rocket/dry_I", I);

        J_inv[0] = 1 / I[0];
        J_inv[1] = 1 / I[1];
        J_inv[2] = 1 / I[2];
    }

    template<typename T>
    inline void generic_rocket_dynamics(const Eigen::Matrix<T, 13, 1> x,
                                        const Eigen::Matrix<T, 3, 1> thrust_vector,
                                        const T torque,
                                        Eigen::Ref <Eigen::Matrix<T, 13, 1>> xdot) const noexcept {
        // -------------- Simulation parameters -------------- -------------
        T g0 = (T) 9.81;                             // Earth gravity in [m/s^2]

        // -------------- Simulation variables -----------------------------
        T mass = (T) dry_mass;

        // Orientation of the rocket with quaternion
        Eigen::Quaternion <T> attitude(x(9), x(6), x(7), x(8));
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Force in inertial frame: gravity
        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T) 0, (T) 0, g0 * mass;

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;
        total_force = rot_matrix * thrust_vector - gravity;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion <T> omega_quat((T) 0.0, x(10), x(11), x(12));

        // X, Y force and Z torque in body frame
        Eigen::Matrix<T, 3, 1> rocket_torque;
        rocket_torque << thrust_vector(0) * CM_to_thrust_distance * (T) J_inv[0],
                thrust_vector(1) * CM_to_thrust_distance * (T) J_inv[1],
                torque * (T) J_inv[2];

        // -------------- Differential equations ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed variation is Force/mass
        xdot.segment(3, 3) = (T) 1e-2 * total_force / mass;

        // Quaternion variation is 0.5*w◦q
        xdot.segment(6, 4) = (T) 0.5 * (omega_quat * attitude).coeffs();

        // Angular speed variation is Torque/Inertia
        xdot.segment(10, 3) = rot_matrix * rocket_torque;
    }

    template<typename T>
    inline void unScaleControl(Eigen::Matrix<T, 4, 1> &u) {
        u(0) = maxServo1Angle * u(0);
        u(1) = maxServo1Angle * u(1);
        u(2) = 0.5 * (u(2) + 1) * (maxThrust - minThrust) + minThrust;
        u(3) = maxTorque * u(3);
    }
};


#endif //SRC_DRONE_MODEL_H
