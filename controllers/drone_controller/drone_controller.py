from controller import Robot
import numpy as np
import struct

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class Mavic(Robot):
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    target_precision = 0.4  # meters

    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())

        # Devices
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.time_step)
        # Motors
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        for motor in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.target = None
        self.target_altitude = 1.5  # Default
        self.got_target = False

    def run(self):
        while self.step(self.time_step) != -1:
            # Receive target only once
            if self.receiver.getQueueLength() > 0 and not self.got_target:
                data = self.receiver.getData()
                self.target = struct.unpack('fff', data)
                print("Drone received target:", self.target)
                self.target_altitude = self.target[2]
                self.got_target = True
                self.receiver.nextPacket()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acc, pitch_acc, _ = self.gyro.getValues()

            if not self.got_target:
                target_xy = [x_pos, y_pos]
                target_alt = altitude
            else:
                target_xy = self.target[:2]
                target_alt = self.target_altitude

            dx = target_xy[0] - x_pos
            dy = target_xy[1] - y_pos
            dist = np.hypot(dx, dy)
            angle_to_target = np.arctan2(dy, dx)
            heading_error = angle_to_target - yaw
            # Normalize to [-pi, pi]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            # Control logic
            yaw_disturbance = clamp(0.6 * heading_error, -0.8, 0.8)
            pitch_disturbance = clamp(-2.5 * dist if abs(heading_error) < np.pi/3 else 0, -1, 1)

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acc
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acc + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_diff_alt = clamp(target_alt - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_diff_alt, 3.0)

            # Set motor velocities (per Webots Mavic2Pro convention)
            fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(fl)
            self.front_right_motor.setVelocity(-fr)
            self.rear_left_motor.setVelocity(-rl)
            self.rear_right_motor.setVelocity(rr)

            # Stop if arrived
            if dist < self.target_precision and abs(target_alt - altitude) < 0.2:
                print(f"Drone reached target ({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})")
                # Optionally: hover or land

robot = Mavic()
robot.run()
