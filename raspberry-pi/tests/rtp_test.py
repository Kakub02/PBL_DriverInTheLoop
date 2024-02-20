import math
import random
import time

from real_time_plot import RealTimePlot

class TemporaryMPU6050:
    def __init__(self, address):
        # Initialize MPU6050 with the provided address
        pass

    def get_accel_data(self):
        # Return random accelerometer data
        return {"x": random.uniform(-10, 10), "y": random.uniform(-1, 1), "z": random.uniform(9, 10)}

    def get_gyro_data(self):
        # Return random gyroscope data
        return {"x": random.uniform(-0.1, 0.1), "y": random.uniform(-0.1, 0.1), "z": random.uniform(-0.1, 0.1)}

RAD_TO_DEG = 57.3  # 180°/pi
g_earth = 9.807
frequency = 50  # in Hz

def measure_imu(rtp_instance: RealTimePlot, number_of_iterations=1000):

    mpu = TemporaryMPU6050(0x68)

    for loop_count in range(number_of_iterations):
        start_time = round(time.time(), 4)

        # Add your MPU6050 data acquisition here
        # Example:
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()

        a_roll_1 = round(math.atan2(accel_data["y"], accel_data["z"]) * RAD_TO_DEG, 2)
        a_pitch_1 = round(math.atan2(accel_data["x"], g_earth) * RAD_TO_DEG, 2)

        a_roll_2 = round(math.atan2(accel_data["y"], math.sqrt(accel_data["x"] ** 2 + accel_data["z"] ** 2)) * RAD_TO_DEG, 2)
        a_pitch_2 = round(math.atan2(accel_data["x"], math.sqrt(accel_data["y"] ** 2 + accel_data["z"] ** 2)) * RAD_TO_DEG, 2)

        with rtp_instance.data_lock:
            rtp_instance.y_data[rtp_instance.lines_labels[0]].append(a_roll_1)
            rtp_instance.y_data[rtp_instance.lines_labels[1]].append(a_pitch_1)
            rtp_instance.y_data[rtp_instance.lines_labels[2]].append(a_roll_2)
            rtp_instance.y_data[rtp_instance.lines_labels[3]].append(a_pitch_2)

        print(start_time, " | a_roll_1 = ", a_roll_1, " | a_pitch_1 = ", a_pitch_1,
                " | a_roll_2 = ", a_roll_2, " | a_pitch_2 = ", a_pitch_2)


        # Add your sleep time calculation here
        sleep_time = 1 / frequency - (time.time() - start_time)  
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

labels = ['acc_roll_1', 'acc_pitch_1', 'acc_roll_2', 'acc_pitch_2']

if __name__ == "__main__":
    real_time_plot = RealTimePlot(labels, 'IMU Measurements over Time', 'Samples', 'Values', 200, (-180, 180), 50, False)
    real_time_plot.set_updating_function(lambda: measure_imu(real_time_plot, 200))
    real_time_plot.run()