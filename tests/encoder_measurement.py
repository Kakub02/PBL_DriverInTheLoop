import time
from encoder_raw import Encoder_raw
import matplotlib.pyplot as plot
import RPi.GPIO as GPIO

left_pin = 16
right_pin = 12  # dostosowac piny


motor_in1 = 5
motor_in2 = 6
motor_ena = 26

class DC_Motor:
    # motor1 = Motor(23, 24, 25)
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
        GPIO.cleanup() # not sure if it should be here

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


def measure_encoder():
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    setpoint_pwm = 0

    frequency = 10  # in Hz
    number_of_measurements = 250

    encoder = Encoder_raw(left_pin, right_pin)

    start_time = time.time()
    elapsed_time = 0
    data_encoder = []
    data_setpoint = []
    data_both = []

    try:
        for loop_count in range(number_of_measurements):
            print(loop_count)
            encoder_value = encoder.getSpeed()

            data_encoder.append({"time": elapsed_time, "value": encoder_value})
            data_setpoint.append({"time": elapsed_time, "value": setpoint_pwm})
            data_both.append({"time": elapsed_time, "encoder_value": encoder_value, "setpoint_pwm": setpoint_pwm})

            time.sleep(1 / frequency)

            elapsed_time = time.time() - start_time
            if loop_count % 50 == 0:
                setpoint_pwm += 10
                motor1.set_speed(setpoint_pwm)

    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script

    #draw_plot(data_encoder, "Time vs Encoder", "Time (s)", "Encoder value", ["value"])
    #draw_plot(data_setpoint, "Time vs Setpoint", "Time (s)", "Setpoint", ["value"])
    draw_plot(data_both, "Time vs Encoder and setpoint", "Time (s)", "Values", ["encoder_value", "setpoint_pwm"])


def draw_plot(data, title, x_label, y_label, legends):
    plot.figure()
    for legend in legends:
        plot.plot([entry["time"] for entry in data], [entry[legend] for entry in data], marker="o", linestyle="-", label=legend)

    plot.title(title)
    plot.xlabel(x_label)
    plot.ylabel(y_label)
    plot.legend()
    plot.show()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    measure_encoder()

    GPIO.cleanup()
