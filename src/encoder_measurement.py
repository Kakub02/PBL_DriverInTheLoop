import time
from encoder import Encoder
from pid import PID
import matplotlib.pyplot as plot
import pandas as pd
import RPi.GPIO as GPIO
import os

left_pin = 16
right_pin = 12  # dostosowac piny


motor_in1 = 5
motor_in2 = 6
motor_ena = 26

max_encoder_value_per_second = 1875
max_encoder_value = 35.935
#18.4
#17.662
#18.426
#19.689333333333334
#19.668
#20.066666666666666
#18.638

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


def measure_max_encoder_speed(number_of_iterations = 1000, start_measuring_iteration = 200, frequency = 100):
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    encoder = Encoder(left_pin, right_pin)

    setpoint_pwm = 100

    total_value = 0
    num_of_readings = 0

    for loop_count in range(number_of_iterations):
        motor1.set_speed(setpoint_pwm)
        encoder_value = encoder.getValueSinceLastRead()
        #encoder_value_per_second = encoder.getSpeed()
        print(loop_count, ": encoder_value = ", encoder_value)

        if loop_count >= start_measuring_iteration:
            total_value += encoder_value
            num_of_readings += 1
        
        time.sleep(1 / frequency)

#TODO measure the time of iteration and sleep only for the rest of the time


    return total_value/num_of_readings

def test_pid(number_of_iterations = 1000, frequency = 100):
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    encoder = Encoder(left_pin, right_pin)

    setpoint_pwm = 70

    pid = PID(1, 0, 0, setpoint=setpoint_pwm)
    pid.output_limits = (0, 100) # pwm borders

    start_time = time.time()
    # last_time = start_time

    elapsed_time = 0
    data_encoder = []
    data_setpoint = []
    data_pid_output = []
    data_both = []

    encoder.startMeasuring()
    for loop_count in range(number_of_iterations):
        pid.setpoint = setpoint_pwm
        encoder_value = encoder.getValueSinceLastRead()
        #encoder_value_per_second = encoder.getSpeed()

        #enc_reading_per_second_in_pwm = encoder_value_per_second/max_encoder_value_per_second * 100
        enc_reading_in_pwm = encoder_value/max_encoder_value * 100

        pid_pwm = pid(encoder_value)
        motor1.set_speed(pid_pwm)
        print(loop_count, " | encoder_value = ", encoder_value, " | enc_in_pwm = ", enc_reading_in_pwm, " | pid_calculated_val = ", pid_pwm)

        data_encoder.append({"time": elapsed_time, "value": enc_reading_in_pwm})
        data_setpoint.append({"time": elapsed_time, "value": setpoint_pwm})
        data_pid_output.append({"time": elapsed_time, "value": pid_pwm})
        # data_both.append({"time": elapsed_time, "encoder_value": enc_reading_in_pwm, "setpoint_pwm": setpoint_pwm})
        data_both.append({"time": elapsed_time, "pid_output": pid_pwm, "encoder_value": enc_reading_in_pwm, "setpoint_pwm": setpoint_pwm})



        elapsed_time = time.time() - start_time
        print(loop_count, " | encoder_value = ", encoder_value, " | enc_in_pwm = ", enc_reading_in_pwm, " | pid_calculated_val = ", pid_pwm, " | iteration_time = ", elapsed_time)
        time.sleep(1 / frequency)
 
    motor1.stop_motor()
    draw_plot(data_encoder, "Time vs Encoder", "Time (s)", "Encoder value", ["value"])
    #draw_plot(data_setpoint, "Time vs Setpoint", "Time (s)", "Setpoint", ["value"])
    draw_plot(data_pid_output, "Time vs PID Output", "Time (s)", "PID Output", ["value"])

    # draw_plot(data_both, "Time vs Encoder and setpoint", "Time (s)", "Values", ["encoder_value", "setpoint_pwm"])
    draw_plot(data_both, "Time vs Encoder, Setpoint, and PID Output", "Time (s)", "Values", ["pid_output", "encoder_value", "setpoint_pwm"])

def measure_encoder(number_of_iterations = 1000, start_measuring_iteration = 200, frequency = 100):
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    setpoint_pwm = 0


    encoder = Encoder(left_pin, right_pin)

    # total_value = 0
    # num_of_readings = 0
    start_time = time.time()
    elapsed_time = 0
    data_encoder = []
    data_setpoint = []
    data_both = []

    try:
        for loop_count in range(number_of_iterations):
            motor1.set_speed(setpoint_pwm)
            encoder_value = encoder.getValueSinceLastRead()
            #encoder_value_per_second = encoder.getSpeed()
            print(loop_count, ": encoder_value = ", encoder_value)

            #enc_reading_per_second_in_pwm = encoder_value_per_second/max_encoder_value_per_second * 100
            enc_reading_in_pwm = encoder_value/max_encoder_value * 100
            data_encoder.append({"time": elapsed_time, "value": enc_reading_in_pwm})
            data_setpoint.append({"time": elapsed_time, "value": setpoint_pwm})
            data_both.append({"time": elapsed_time, "encoder_value": enc_reading_in_pwm, "setpoint_pwm": setpoint_pwm})

            time.sleep(1 / frequency)

            elapsed_time = time.time() - start_time

            if loop_count == 100:
                setpoint_pwm = 50
                print("setpoint_pwm = ", setpoint_pwm)
            # if loop_count >= start_measuring_iteration:
            #     total_value += encoder_value
            #     num_of_readings += 1

    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script
    #print(total_value/num_of_readings)
    motor1.stop_motor()
    draw_plot(data_encoder, "Time vs Encoder", "Time (s)", "Encoder value", ["value"])
    draw_plot(data_setpoint, "Time vs Setpoint", "Time (s)", "Setpoint", ["value"])
    draw_plot(data_both, "Time vs Encoder and setpoint", "Time (s)", "Values", ["encoder_value", "setpoint_pwm"])
    createXLSX(data_both, "encoder_setpoint_over_time.xlsx")


def draw_plot(data, title, x_label, y_label, legends):
    plot.figure()
    for legend in legends:
        plot.plot([entry["time"] for entry in data], [entry[legend] for entry in data], marker="o", linestyle="-", label=legend)

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

    #test_pid(2000, 50)
    
    #na poczatku to
    #measure_max_encoder_speed(2000,500, 50)
    print(measure_max_encoder_speed(2000,500, 50))
    #potem to
    #measure_encoder(2000,500, 50)

    GPIO.cleanup()
