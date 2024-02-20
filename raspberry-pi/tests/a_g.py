import math as m
import time

from mpu6050 import mpu6050
from real_time_plot import RealTimePlot

alpha = 0.98
frequency = 50  # in Hz
dt = 1 / frequency

mpu = mpu6050(0x68)

def measure_imu(rtp_instance: RealTimePlot, number_of_iterations=1000):
    
    #Initial accelerometer angle
    accel_data = mpu.get_accel_data()
    a_roll = round(m.degrees(m.atan2(accel_data["y"], m.sqrt(accel_data["x"] ** 2 + accel_data["z"] ** 2))), 2)
    a_pitch = round(m.degrees(m.atan2((-1)*accel_data["x"], m.sqrt(accel_data["y"] ** 2 + accel_data["z"] ** 2))), 2)

    #Initial gyroscope angle - set to accelerometer initial angle. This ensures a smoother transition at the beginning.
    g_roll = a_roll
    g_pitch = a_pitch

    #Initial final angle - angle after applying the complementary filter
    final_roll = a_roll
    final_pitch = a_pitch

    for _ in range(number_of_iterations):
        start_time = time.time()

        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()

        # Accelerometer-derived angles
        a_roll = round(m.degrees(m.atan2(accel_data["y"], m.sqrt(accel_data["x"] ** 2 + accel_data["z"] ** 2))), 2)
        a_pitch = round(m.degrees(m.atan2((-1)*accel_data["x"], m.sqrt(accel_data["y"] ** 2 + accel_data["z"] ** 2))), 2)

        # Gyroscope-derived angles
        g_roll = gyro_data["x"] * dt
        g_pitch =   gyro_data["y"] * dt

        # Complementary filter
        final_roll = alpha * (final_roll + g_roll) + (1 - alpha) * a_roll
        final_pitch = alpha * (final_pitch + g_pitch) + (1 - alpha) * a_pitch


        with rtp_instance.data_lock:
            rtp_instance.y_data[rtp_instance.lines_labels[0]].append(a_roll)
            rtp_instance.y_data[rtp_instance.lines_labels[1]].append(a_pitch)
            rtp_instance.y_data[rtp_instance.lines_labels[2]].append(g_roll)
            rtp_instance.y_data[rtp_instance.lines_labels[3]].append(g_pitch)
            rtp_instance.y_data[rtp_instance.lines_labels[4]].append(final_roll)
            rtp_instance.y_data[rtp_instance.lines_labels[5]].append(final_pitch)

        print(round(start_time, 4), " | a_roll = ", a_roll, " | a_pitch = ", a_pitch,
                                    " | g_roll = ", g_roll, " | g_pitch = ", g_pitch,
                                    " | final_roll = ", final_roll, " | final_pitch = ", final_pitch)

        # Calculating sleep time so the frequency is maintained
        sleep_time = 1 / frequency - (time.time() - start_time)  
        if sleep_time > 0:
            time.sleep(sleep_time)

labels = ['a_roll', 'a_pitch', 'g_roll', 'g_pitch', 'final_roll', 'final_pitch']
# final_labels = ['roll', 'pitch']

if __name__ == "__main__":

    real_time_plot = RealTimePlot(labels, 'IMU Measurements over Time', 'Samples', 'Values', 200, (-180, 180), 50, False)
    real_time_plot.set_updating_function(lambda: measure_imu(real_time_plot, 10000))
    real_time_plot.run()
