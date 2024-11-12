import json
import time
import numpy as np
from odrive_uart import ODriveUART
from imu import FilteredLSM6DS3

def test_motor_direction():
    # Initialize IMU and motors
    imu = FilteredLSM6DS3()
    imu.calibrate()
    
    right_motor = ODriveUART(axis_num=0, dir=1)
    left_motor = ODriveUART(axis_num=1, dir=1)
    
    directions = {'left': 1, 'right': 1}
    
    # Test each motor
    for motor, name in [(left_motor, 'left'), (right_motor, 'right')]:
        print(f"\nTesting {name} motor...")
        
        # Start motor in closed loop control
        motor.start()
        motor.enable_velocity_mode()
        
        # Get baseline gyro reading
        imu.update()
        baseline_gyro = imu.gyro_RAW[2]  # Z-axis rotation
        
        # Spin motor
        motor.set_speed_rpm(60)
        time.sleep(0.5)
        
        # Get gyro reading during spin
        imu.update()
        spin_gyro = imu.gyro_RAW[0]
        
        # Stop motor
        motor.stop()
        
        # Determine direction based on gyro reading
        # Positive gyro means counterclockwise rotation when viewed from above
        gyro_diff = spin_gyro - baseline_gyro
        print(f"Gyro difference: {gyro_diff}")
        
        # Set direction based on gyro reading
        # We want positive direction to be forward motion
        directions[name] = -1 if gyro_diff > 0 else 1
        
        time.sleep(0.5)  # Wait between tests
    
    # Save results to file
    with open('motor_dir.json', 'w') as f:
        json.dump(directions, f)
    
    print("\nDirection test complete!")
    print(f"Results saved to motor_dir.json: {directions}")

if __name__ == '__main__':
    try:
        test_motor_direction()
    except Exception as e:
        print(f"Error occurred: {e}")
