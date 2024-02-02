import time
import matplotlib.pyplot as plt
import pandas as pd
import os

from adafruit_servokit import ServoKit
from mpu6050 import mpu6050
from pid_controller import PID_controller
from encoder_measurement import createXLSX, draw_plot

roll_pin = 0
pitch_pin = 1

frequency = 50  # in Hz

def measure_imu(number_of_iterations=1000):
    pca9685 = ServoKit(channels=16, address=0x40)  # PCA9685
    mpu = mpu6050(0x68)

    setpoint_roll_angle = 90
    setpoint_pitch_angle = 90

    data_all = []
    plt.ion()  # Enable interactive mode for real-time plotting

    try:
        for loop_count in range(number_of_iterations):
            start_time = time.time()

            accel_data = mpu.get_accel_data()
            gyro_data = mpu.get_gyro_data()

            pca9685.servo[roll_pin].angle = setpoint_roll_angle 
            pca9685.servo[pitch_pin].angle = setpoint_pitch_angle

            print(loop_count, "| acc_x = ", accel_data["x"], "| acc_y = ", accel_data["y"], "| acc_z = ", accel_data["z"], "| gyr_x = ", gyro_data["x"],"| gyr_y = ", gyro_data["y"],"| gyr_z = ", gyro_data["z"])


            data_all.append({"time": start_time, "acc_x": accel_data["x"], "acc_y": accel_data["y"], "acc_z": accel_data["z"], "gyr_x": gyro_data["x"], "gyr_y": gyro_data["y"], "gyr_z": gyro_data["z"]})

            if loop_count == 100:
                setpoint_roll_angle = 120
                setpoint_pitch_angle = 120
                print("sp_pitch = ", setpoint_pitch_angle, "sp_roll = ", setpoint_roll_angle)


            draw_plot(data_all, "Time vs Accelerometer and Gyroscope", "Time (s)", "Values", ["acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z"])

            sleep_time = 1 / frequency - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()  # Turn off interactive mode when done
        plt.show()

    pca9685.servo[roll_pin].angle = 90 
    pca9685.servo[pitch_pin].angle = 90

if __name__ == "__main__":
    measure_imu()
