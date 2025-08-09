// controllers/mavic/mavic.c



#include <math.h>

#include <stdio.h>

#include <stdlib.h>

#include <string.h>



#ifndef M_PI

#define M_PI 3.14159265358979323846

#endif



#include <webots/robot.h>

#include <webots/camera.h>

#include <webots/compass.h>

#include <webots/gps.h>

#include <webots/gyro.h>

#include <webots/inertial_unit.h>

#include <webots/keyboard.h>

#include <webots/led.h>

#include <webots/motor.h>



#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))



int main(int argc, char **argv) {

    wb_robot_init();

    int timestep = (int)wb_robot_get_basic_time_step();

    const char *robot_name = wb_robot_get_name();



    // Devices

    WbDeviceTag camera = wb_robot_get_device("camera");

    wb_camera_enable(camera, timestep);

    WbDeviceTag imu = wb_robot_get_device("inertial unit");

    wb_inertial_unit_enable(imu, timestep);

    WbDeviceTag gps = wb_robot_get_device("gps");

    wb_gps_enable(gps, timestep);

    WbDeviceTag compass = wb_robot_get_device("compass");

    wb_compass_enable(compass, timestep);

    WbDeviceTag gyro = wb_robot_get_device("gyro");

    wb_gyro_enable(gyro, timestep);

    WbDeviceTag motors[4];

    const char *motor_names[4] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};

    for (int i = 0; i < 4; ++i) {

        motors[i] = wb_robot_get_device(motor_names[i]);

        wb_motor_set_position(motors[i], INFINITY);

        wb_motor_set_velocity(motors[i], 1.0);

    }



    while (wb_robot_step(timestep) != -1) {

        if (wb_robot_get_time() > 1.0) break;

    }



    // --- PID and Flight Constants ---

    const double k_vertical_thrust = 68.5;

    const double k_vertical_offset = 0.6;

    // *** CHANGE 1: Reduced vertical gain ***

    const double k_vertical_p = 2.0;

    const double k_roll_p = 50.0;

    const double k_pitch_p = 30.0;

    const double k_yaw_p = 0.3;

    const double target_altitude = 1.5;



    int print_counter = 0;

    double yaw_diff = 0.0;



    while (wb_robot_step(timestep) != -1) {

        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);

        const double roll = rpy[0];

        const double pitch = rpy[1];

        const double yaw = rpy[2];

        const double *gps_values = wb_gps_get_values(gps);

        const double roll_velocity = wb_gyro_get_values(gyro)[0];

        const double pitch_velocity = wb_gyro_get_values(gyro)[1];

        

        const char *custom_data_str = wb_robot_get_custom_data();

        double target_x = gps_values[0];

        double target_y = gps_values[1];

        if (strlen(custom_data_str) > 0) {

            sscanf(custom_data_str, "%lf %lf", &target_x, &target_y);

        }



        double roll_disturbance = 0.0;

        double pitch_disturbance = 0.0;

        double yaw_disturbance = 0.0;



        double dx = target_x - gps_values[0];

        double dy = target_y - gps_values[1];

        double distance_to_target = sqrt(dx * dx + dy * dy);

        double target_yaw = atan2(dy, dx);

        

        if (distance_to_target > 0.2) {

            yaw_diff = target_yaw - yaw;

            while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

            yaw_disturbance = k_yaw_p * yaw_diff;



            if (fabs(yaw_diff) < 0.4) { 

                pitch_disturbance = -2.0;

            }

        }

        

        const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;

        const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;

        

        // --- Altitude Control Calculation ---

        const double clamped_diff_alt = CLAMP(target_altitude - gps_values[2] + k_vertical_offset, -1.0, 1.0);

        // *** CHANGE 2: Removed pow(..., 3.0) for a linear, more stable response ***

        const double vertical_input = k_vertical_p * clamped_diff_alt;

        

        // --- Actuate Motors ---

        const double front_left = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;

        const double front_right = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;

        const double rear_left = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;

        const double rear_right = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;

        

        wb_motor_set_velocity(motors[0], front_left);

        wb_motor_set_velocity(motors[1], -front_right);

        wb_motor_set_velocity(motors[2], -rear_left);

        wb_motor_set_velocity(motors[3], rear_right);



        if (print_counter++ > 100) {

            printf("[%s] Target: (%.2f, %.2f) | Current: (%.2f, %.2f) | Dist: %.2f | Yaw Diff: %.2f\n",

                   robot_name, target_x, target_y, gps_values[0], gps_values[1], distance_to_target, yaw_diff);

            print_counter = 0;

        }

    };



    wb_robot_cleanup();

    return EXIT_SUCCESS;

}