import time
from mpu6050 import mpu6050
import matplotlib.pyplot as plot


def measure_imu():
    frequency = 10  # in Hz
    number_of_measurements = 500

    mpu = mpu6050(0x68)

    start_time = time.time()
    elapsed_time = 0
    data_x = []
    data_y = []

    try:
        for _ in range(number_of_measurements):
            accel_data = mpu.get_accel_data()

            data_x.append({"time": elapsed_time, "value": accel_data["x"]})
            data_y.append({"time": elapsed_time, "value": accel_data["y"]})

            time.sleep(1 / frequency)

            elapsed_time = time.time() - start_time
    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script

    draw_plot(data_x, "Time vs Accelerometer X", "Time (s)", "Accel X (chuj wie jaka jednostka)")
    draw_plot(data_y, "Time vs Accelerometer Y", "Time (s)", "Accel Y (chuj wie jaka jednostka)")


def draw_plot(data, title, x_label, y_label):
    plot.figure()
    plot.plot([entry["time"] for entry in data], [entry["value"] for entry in data], marker="o", linestyle="-", color="b")

    plot.title(title)
    plot.xlabel(x_label)
    plot.ylabel(y_label)

    plot.show()


if __name__ == "__main__":
    measure_imu()
