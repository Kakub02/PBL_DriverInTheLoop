import time
from encoder_raw import Encoder_raw
import matplotlib.pyplot as plot
import RPi.GPIO as GPIO

left_pin = 69
right_pin = 69  # dostosowac piny


def measure_encoder():
    frequency = 10  # in Hz
    number_of_measurements = 500

    encoder = Encoder_raw(left_pin, right_pin)

    start_time = time.time()
    elapsed_time = 0
    data_encoder = []

    try:
        for _ in range(number_of_measurements):
            encoder_value = encoder.getValue()

            data_encoder.append({"time": elapsed_time, "value": encoder_value})

            time.sleep(1 / frequency)

            elapsed_time = time.time() - start_time

    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script

    draw_plot(data_encoder, "Time vs Encoder", "Time (s)", "Encoder value")


def draw_plot(data, title, x_label, y_label):
    plot.figure()
    plot.plot([entry["time"] for entry in data], [entry["value"] for entry in data], marker="o",linestyle="-", color="b")

    plot.title(title)
    plot.xlabel(x_label)
    plot.ylabel(y_label)

    plot.show()


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    measure_encoder()

    GPIO.cleanup()
