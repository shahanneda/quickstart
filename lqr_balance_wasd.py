import time
import traceback
import os
import numpy as np
import matplotlib.pyplot as plt
from imu import FilteredLSM6DS3
from odrive_uart import ODriveUART
from lqr import LQR_gains
import json
from threading import Thread
from sshkeyboard import stop_listening, listen_keyboard
from RPi import GPIO

# GPIO setup for resetting ODrive
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)

# Constants from original file
WHEEL_RADIUS = 6.5 * 0.0254 / 2
WHEEL_DIST = 0.235
YAW_RATE_TO_MOTOR_TORQUE = (WHEEL_DIST / WHEEL_RADIUS) * 0.1
MOTOR_TURNS_TO_LINEAR_POS = WHEEL_RADIUS * 2 * np.pi
RPM_TO_METERS_PER_SECOND = WHEEL_RADIUS * 2 * np.pi / 60
MAX_TORQUE = 2.5
MAX_SPEED = 1.0

class KeyboardThread(Thread):
    def __init__(self):
        super().__init__()
        self.pressed_key = None
        self.debounce_timer = None
        self.debounce_delay = 0.0
        self.desired_vel = 0
        self.desired_yaw_rate = 0
        self.zero_angle_adjustment = 0

    def press(self, key):
        if self.debounce_timer:
            self.debounce_timer.cancel()
            self.debounce_timer = None
        
        if key == self.pressed_key:
            return
        
        self.pressed_key = key

        if key == 'w':
            self.desired_vel = MAX_SPEED / 2
        elif key == 's':
            self.desired_vel = -MAX_SPEED / 2
        elif key == 'a':
            self.desired_yaw_rate = -2.0  # ANGULAR_SPEED
        elif key == 'd':
            self.desired_yaw_rate = 2.0  # -ANGULAR_SPEED
        elif key == 'up':
            self.zero_angle_adjustment = 0.025
        elif key == 'down':
            self.zero_angle_adjustment = -0.025
        elif key == 'q':
            print("Quit")
            stop_listening()
        # print(key.upper())

    def release(self, key):
        if key != self.pressed_key:
            return
        
        if self.debounce_timer:
            self.debounce_timer.cancel()
        
        if key in ['w', 's']:
            self.desired_vel = 0
        elif key in ['a', 'd']:
            self.desired_yaw_rate = 0
        elif key in ['up', 'down']:
            self.zero_angle_adjustment = 0
        
        self.pressed_key = None

    def run(self):
        try:
            print("Use WASD to control the robot. Up/Down arrows to adjust balance. Press 'q' to quit.")
            listen_keyboard(
                on_press=self.press,
                on_release=self.release,
                sequential=False,
                delay_second_char=0
            )
        except Exception as e:
            print(e)

def reset_odrive():
    GPIO.output(5, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(5, GPIO.HIGH)
    print("ODrive reset attempted")


def balance():
    # Initialize keyboard thread
    keyboard_control = KeyboardThread()
    keyboard_control.start()

    # Initialize IMU and LQR gains (same as original)
    imu = FilteredLSM6DS3()
    K_balance = LQR_gains(Q_diag=[100,10,100,1,10,1], R_diag=[0.2, 1])
    K_drive = LQR_gains(Q_diag=[0,100,1,1,1,10], R_diag=[0.1, 1])
    print(K_balance.round(2))
    Dt = 1./400.

    # Initialize variables
    zero_angle = -1
    start_plot_time = time.time()

    # Initialize plotting arrays (same as original)
    times, positions, desired_positions = [], [], []
    velocities, desired_velocities = [], []
    pitches, desired_pitches = [], []
    pitch_rates, desired_pitch_rates = [], []
    yaws, desired_yaws = [], []
    yaw_rates, desired_yaw_rates = [], []

    # Reset ODrive and initialize motors
    reset_odrive()
    time.sleep(1)

    # Initialize motors (same as original)
    try:
        with open('motor_dir.json', 'r') as f:
            motor_dirs = json.load(f)
            left_dir = motor_dirs['left']
            right_dir = motor_dirs['right']
    except Exception as e:
        raise Exception("Error reading motor_dir.json")

    right_motor = ODriveUART(axis_num=0, dir=right_dir)
    left_motor = ODriveUART(axis_num=1, dir=left_dir)
    left_motor.enable_torque_mode()
    right_motor.enable_torque_mode()
    left_motor.start()
    right_motor.start()

    def reset_and_initialize_motors():
        nonlocal left_motor, right_motor
        reset_odrive()
        time.sleep(1)  # Give ODrive time to reset
        try:
            right_motor = ODriveUART(axis_num=0, dir=right_dir)
            left_motor = ODriveUART(axis_num=1, dir=left_dir)
            right_motor.clear_errors()
            left_motor.clear_errors()
            left_motor.enable_torque_mode()
            right_motor.enable_torque_mode()
            left_motor.start()
            right_motor.start()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    # Record starting position
    try:
        l_pos = left_motor.get_position_turns()
        r_pos = right_motor.get_position_turns()
        start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
        start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)
    except Exception as e:
        print(f"Error reading initial motor positions: {e}")
        return

    imu.calibrate()
    cycle_count = 0
    is_pos_control = True

    try:
        left_motor.enable_watchdog()
        right_motor.enable_watchdog()
        
        while True:
            loop_start_time = time.time()

            if cycle_count % 20 == 0:
                try:
                    left_error_code, left_error = left_motor.get_errors()
                    right_error_code, right_error = right_motor.get_errors()
                    if left_error_code != 0 or right_error_code != 0:
                        print(f"Detected ODriveUART errors - Left: {left_error_code} ({left_error}), Right: {right_error_code} ({right_error})")
                        reset_and_initialize_motors()
                        continue
                except Exception as e:
                    print('Error checking motor errors:', e)
                    reset_and_initialize_motors()
                    continue

            # Get desired velocity and yaw rate from keyboard thread
            desired_vel = keyboard_control.desired_vel
            desired_yaw_rate = keyboard_control.desired_yaw_rate
            # zero_angle += keyboard_control.zero_angle_adjustment

            was_pos_control = is_pos_control
            # is_pos_control = False
            is_pos_control = desired_vel == 0 and desired_yaw_rate == 0 and np.mean(np.abs([0]+velocities[-50:])) < 0.2
            if is_pos_control and not was_pos_control:
                start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            if desired_yaw_rate != 0:
                start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)

            # Rest of the control loop (same as original)
            current_pitch = imu.robot_angle() 
            current_yaw_rate = -imu.gyro_RAW[2]
            current_pitch_rate = imu.gyro_RAW[0]


            try:
                l_pos, l_vel = left_motor.get_pos_vel() 
                r_pos, r_vel = right_motor.get_pos_vel() 
                current_vel = (l_vel + r_vel) / 2 * RPM_TO_METERS_PER_SECOND
                current_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS - start_pos
                current_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST) - start_yaw
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue

            if is_pos_control and abs(current_vel) < 0.01:
                zero_angle += 0.0002*np.sign(current_pitch-zero_angle)

            # Store data for plotting
            current_time = time.time()
            times.append(current_time - start_plot_time)
            positions.append(current_pos)
            desired_positions.append(0)
            velocities.append(current_vel)
            desired_velocities.append(desired_vel)
            pitches.append(current_pitch)
            desired_pitches.append(zero_angle)
            pitch_rates.append(current_pitch_rate)
            desired_pitch_rates.append(0)
            yaws.append(current_yaw)
            desired_yaws.append(0)
            yaw_rates.append(current_yaw_rate)
            desired_yaw_rates.append(desired_yaw_rate)

            # Calculate control outputs
            current_state = np.array([
                current_pos, current_vel, current_pitch*np.pi/180, 
                current_pitch_rate, current_yaw, current_yaw_rate
            ])
            
            desired_state = np.array([
                0, desired_vel, zero_angle*np.pi/180, 0, 0, desired_yaw_rate
            ])

            state_error = (current_state - desired_state).reshape((6,1))

            if is_pos_control:
                # Position control
                C = -K_balance @ state_error
            else:
                # Velocity control
                state_error[0,0] = 0
                state_error[4,0] = 0
                C = -K_drive @ state_error
            
                
            D = np.array([[0.5,0.5],[0.5,-0.5]])
            left_torque, right_torque = (D @ C).squeeze()

            # Limit torques
            left_torque = np.clip(left_torque, -MAX_TORQUE, MAX_TORQUE)
            right_torque = np.clip(right_torque, -MAX_TORQUE, MAX_TORQUE)

            # Apply torques
            try:
                left_motor.set_torque_nm(left_torque)
                right_motor.set_torque_nm(right_torque)
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue

            if cycle_count % 50 == 0:
                print(f"Loop time: {time.time() - loop_start_time:.6f} sec, x=[{current_pos:.2f} | 0], v=[{current_vel:.2f} | {desired_vel:.2f}], θ=[{current_pitch:.2f} | {zero_angle:.2f}], ω=[{current_pitch_rate:.2f} | 0], δ=[{current_yaw:.2f} | 0], δ'=[{current_yaw_rate:.2f} | {desired_yaw_rate:.2f}]")

            cycle_count += 1
            time.sleep(max(0, Dt - (time.time() - current_time)))

    except KeyboardInterrupt:
        print("Balance stopped by user.")

    except Exception as e:
        print("An error occurred:")
        traceback.print_exc()

    finally:
        # Cleanup
        stop_listening()
        keyboard_control.join()
        left_motor.disable_watchdog()
        right_motor.disable_watchdog()
        left_motor.stop()
        right_motor.stop()

        # Create the plots
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(15, 12))
        
        # Plot positions (x)
        ax1.plot(times, positions, label='Current Position')
        ax1.plot(times, desired_positions, label='Desired Position')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position (m)')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title('Position')
        
        # Plot velocities (v)
        ax2.plot(times, velocities, label='Current Velocity')
        ax2.plot(times, desired_velocities, label='Desired Velocity')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.legend()
        ax2.grid(True)
        ax2.set_title('Velocity')

        # Plot pitches
        ax3.plot(times, pitches, label='Current Pitch')
        ax3.plot(times, desired_pitches, label='Desired Pitch')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Pitch (deg)')
        ax3.legend()
        ax3.grid(True)
        ax3.set_title('Pitch')

        # Plot pitch rates
        ax4.plot(times, pitch_rates, label='Current Pitch Rate')
        ax4.plot(times, desired_pitch_rates, label='Desired Pitch Rate')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Pitch Rate (rad/s)')
        ax4.legend()
        ax4.grid(True)
        ax4.set_title('Pitch Rate')
        ax4.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax4.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        # Plot yaws
        ax5.plot(times, yaws, label='Current Yaw')
        ax5.plot(times, desired_yaws, label='Desired Yaw')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Yaw (rad)')
        ax5.legend()
        ax5.grid(True)
        ax5.set_title('Yaw')
        ax5.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax5.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        # Plot yaw rates
        ax6.plot(times, yaw_rates, label='Current Yaw Rate')
        ax6.plot(times, desired_yaw_rates, label='Desired Yaw Rate')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Yaw Rate (rad/s)')
        ax6.legend()
        ax6.grid(True)
        ax6.set_title('Yaw Rate')
        ax6.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax6.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        plt.tight_layout()
        plt.savefig('plots.png')
        os.system('cursor plots.png')

if __name__ == "__main__":
    balance()