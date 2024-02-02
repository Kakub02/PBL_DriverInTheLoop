import time

# from encoder import Encoder
from enc_test import Encoder_
# from pid import PID
from pid_controller import PID_controller
import matplotlib.pyplot as plot
import pandas as pd
import RPi.GPIO as GPIO
import os

left_pin = 16
right_pin = 12

motor_in1 = 5
motor_in2 = 6
motor_ena = 26

max_carla_speed = 60

max_encoder_speed = 10000  # v max of encoder in impulses/s

frequency = 50

class DC_Motor:
    def __init__(self, in1, in2, en):
        self.in1 = in1
        self.in2 = in2
        self.en = en

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 1000)
        self.pwm.start(0)

    def __del__(self):
        self.pwm.stop()

    def set_speed(self, setPointSpeed=0, reverse=0):
        if reverse == 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif reverse == 1:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(setPointSpeed)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)


def measure_max_encoder_speed(number_of_iterations=1000, start_measuring_iteration=200):
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    encoder = Encoder_(left_pin, right_pin)

    setpoint_pwm = 100

    total_value = 0
    num_of_readings = 0

    for loop_count in range(number_of_iterations):
        t1 = time.time()

        encoder_value = encoder.getValue()
        encoder.clearValue()
        motor1.set_speed(setpoint_pwm)
        print(loop_count, ": encoder_value = ", encoder_value)

        if loop_count >= start_measuring_iteration:
            total_value += encoder_value
            num_of_readings += 1

        sleep_time = 1 / frequency - (time.time() - t1)
        if sleep_time > 0:
            time.sleep(sleep_time)

    return total_value / num_of_readings * frequency


def test_pid(number_of_iterations=1000):
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    encoder = Encoder_(left_pin, right_pin)

    pid = PID_controller(0.1538, 2.263, 0, 1, 0, 100, 0, 250, 1/frequency)

    setpoint_speed = 0 * max_encoder_speed

    elapsed_time = 0
    data_both = []

    try:
        for loop_count in range(number_of_iterations):
            t1 = time.time()

            encoder_value = encoder.getValue()
            encoder.clearValue()

            real_speed = encoder_value * frequency
            pid_pwm = pid.update(setpoint_speed, real_speed)
            motor1.set_speed(pid_pwm)
            
            print(loop_count, " | setpoint_speed = ", setpoint_speed, " | real_speed = ", real_speed, " | pid_output = ", pid_pwm)
            
            data_both.append({"time": t1, "encoder_speed": real_speed, "setpoint_speed": setpoint_speed})

            if loop_count == 100:
                setpoint_speed = 0.5 * max_encoder_speed
            if loop_count == 700:
                setpoint_speed = 0.8 * max_encoder_speed

            sleep_time = 1 / frequency - (time.time() - t1)
            if sleep_time > 0:
                elapsed_time = 1 / frequency
                time.sleep(sleep_time)
            else:
                elapsed_time = time.time() - t1

    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script
    motor1.stop_motor()
    draw_plot(data_both, "Time vs Encoder and setpoint", "Time (s)", "Values", ["encoder_speed", "setpoint_speed"])
    createXLSX(data_both, "dc_with_pid.xlsx")

def measure_encoder(number_of_iterations=1000):
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    encoder = Encoder_(left_pin, right_pin)

    setpoint_pwm = 0

    elapsed_time = 0
    data_encoder = []
    data_setpoint = []
    data_elapsedtime = []
    data_both = []

    try:
        for loop_count in range(number_of_iterations):
            t1 = time.time()

            encoder_value = encoder.getValue()
            encoder.clearValue()
            motor1.set_speed(setpoint_pwm)
            print(loop_count, ": encoder_value = ", encoder_value)

            # enc_reading_per_second_in_pwm = encoder_value_per_second/max_encoder_value_per_second * 100
            real_speed = encoder_value * frequency

            data_encoder.append({"time": t1, "value": real_speed})
            data_setpoint.append({"time": t1, "value": setpoint_pwm*max_encoder_speed/100})
            #data_elapsedtime.append({"time": t1, "value": elapsed_time})
            data_both.append({"time": t1, "encoder_speed": real_speed, "setpoint_speed": setpoint_pwm*max_encoder_speed/100})

            if loop_count == 100:
                setpoint_pwm = 50
                print("setpoint_pwm = ", setpoint_pwm)

            sleep_time = 1 / frequency - (time.time() - t1)
            if sleep_time > 0:
                elapsed_time = 1 / frequency
                time.sleep(sleep_time)
            else:
                elapsed_time = time.time() - t1

    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script
    motor1.stop_motor()
    #draw_plot(data_encoder, "Time vs Encoder_speed", "Time (s)", "Encoder speed", ["value"])
    #draw_plot(data_setpoint, "Time vs Setpoint", "Time (s)", "Setpoint", ["value"])
    #draw_plot(data_elapsedtime, "Time vs elapsed_time", "Time (s)", "elapsed", ["value"])
    draw_plot(data_both, "Time vs Encoder and setpoint", "Time (s)", "Values", ["encoder_speed", "setpoint_speed"])
    createXLSX(data_both, "encoder_setpoint_over_time.xlsx")


def draw_plot(data, title, x_label, y_label, legends):
    plot.figure()
    for legend in legends:
        plot.plot(
            [entry["time"] for entry in data],
            [entry[legend] for entry in data],
            marker="o",
            linestyle="-",
            label=legend,
        )

    plot.title(title)
    plot.xlabel(x_label)
    plot.ylabel(y_label)
    plot.legend()
    plot.show()


def createXLSX(data, filename):
    # If the file exists, modify the filename
    if os.path.exists(filename):
        base, ext = os.path.splitext(filename)
        counter = 1
        while os.path.exists(f"{base}_{counter}{ext}"):
            counter += 1
        filename = f"{base}_{counter}{ext}"

    # Create the Excel file
    df = pd.DataFrame(data)
    df.to_excel(filename, index=False)
    print(f"Excel file '{filename}' created successfully.")

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    # na poczatku to
    #print(measure_max_encoder_speed(2000,400))
    # potem to
    #measure_encoder(2000)
    # na koncu to
    test_pid()

    GPIO.cleanup()
