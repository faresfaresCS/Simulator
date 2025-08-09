// controllers/mavic/mavic.c - FINAL AUTONOMOUS VERSION

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/led.h>
#include <webots/motor.h>

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
    wb_robot_init();
    int timestep = (int)wb_robot_get_basic_time_step();

    // --- Get target from controller arguments ---
    double target_x = 0.0;
    double target_y = 0.0;
    if (argc > 2) {
      target_x = atof(argv[1]);
      target_y = atof(argv[2]);
      printf("[%s] New target: (%.2f, %.2f)\n", wb_robot_get_name(), target_x, target_y);
    } else {
      printf("[%s] No target arguments received.\n", wb_robot_get_name());
    }

    // --- Devices (from original script) ---
    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, timestep);
    WbDeviceTag front_left_led = wb_robot_get_device("front left led");
    WbDeviceTag front_right_led = wb_robot_get_device("front right led");
    WbDeviceTag imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, timestep);
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, timestep);
    WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
    WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
    WbDeviceTag motors[4];
    const char *motor_names[4] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};
    for (int i = 0; i < 4; ++i) {
        motors[i] = wb_robot_get_device(motor_names[i]);
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 1.0);
    }

    // --- Constants (from original script) ---
    const double k_vertical_thrust = 68.5;
    const double k_vertical_offset = 0.6;
    const double k_vertical_p = 3.0;
    const double k_roll_p = 50.0;
    const double k_pitch_p = 30.0;
    const double k_yaw_p = 1.5; // Yaw gain to turn towards target
    const double target_altitude = 1.0;

    // --- Main loop ---
    while (wb_robot_step(timestep) != -1) {
        const double time = wb_robot_get_time();
        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
        const double roll = rpy[0];
        const double pitch = rpy[1];
        const double yaw = rpy[2];
        const double altitude = wb_gps_get_values(gps)[2];
        const double *gps_values = wb_gps_get_values(gps);
        const double roll_velocity = wb_gyro_get_values(gyro)[0];
        const double pitch_velocity = wb_gyro_get_values(gyro)[1];

        // --- Visuals ---
        const bool led_state = ((int)time) % 2;
        wb_led_set(front_left_led, led_state);
        wb_led_set(front_right_led, !led_state);
        wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
        wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);

        // --- Autonomous Navigation Logic ---
        double roll_disturbance = 0.0;
        double pitch_disturbance = 0.0;
        double yaw_disturbance = 0.0;

        double dx = target_x - gps_values[0];
        double dy = target_y - gps_values[1];
        
        if (sqrt(dx * dx + dy * dy) > 0.2) {
            double target_yaw = atan2(dy, dx);
            double yaw_diff = target_yaw - yaw;
            //Normalize angle
            while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
            
            yaw_disturbance = k_yaw_p * yaw_diff;
            
            if (fabs(yaw_diff) < 0.2) {
                pitch_disturbance = -2.0; // Move forward
            }
        }
        
        // --- Core stabilization (from original script) ---
        const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        const double yaw_input = yaw_disturbance;
        const double clamped_difference_altitude = CLAMP(target_altitude - gps_values[2] + k_vertical_offset, -1.0, 1.0);
        const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

        // --- Motor mixing (from original script) ---
        const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
        const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
        const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
        const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
        
        // --- Final actuation (from original script) ---
        wb_motor_set_velocity(motors[0], front_left_motor_input);
        wb_motor_set_velocity(motors[1], -front_right_motor_input);
        wb_motor_set_velocity(motors[2], -rear_left_motor_input);
        wb_motor_set_velocity(motors[3], rear_right_motor_input);
    };

    wb_robot_cleanup();
    return EXIT_SUCCESS;
}