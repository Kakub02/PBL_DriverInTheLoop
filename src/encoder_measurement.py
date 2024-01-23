import time
from encoder import Encoder
from pid import PID
import matplotlib.pyplot as plot
import RPi.GPIO as GPIO

left_pin = 16
right_pin = 12  # dostosowac piny


motor_in1 = 5
motor_in2 = 6
motor_ena = 26

max_encoder_value_per_second = 1875
max_encoder_value = 24.29

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

    return total_value/num_of_readings

def test_pid():
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    encoder = Encoder(left_pin, right_pin)

    setpoint_pwm = 50

    pid = PID(5, 0.01, 0.1, setpoint=setpoint_pwm)
    pid.output_limits = (0, 100) # pwm borders

    frequency = 100 # in Hz
    number_of_iterations = 1000

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
        data_both.append({"time": elapsed_time, "encoder_value": enc_reading_in_pwm, "setpoint_pwm": setpoint_pwm, "pid_output": pid_pwm})

        time.sleep(1 / frequency)

        elapsed_time = time.time() - start_time
    

 
    motor1.stop_motor()
    draw_plot(data_encoder, "Time vs Encoder", "Time (s)", "Encoder value", ["value"])
    draw_plot(data_setpoint, "Time vs Setpoint", "Time (s)", "Setpoint", ["value"])
    draw_plot(data_pid_output, "Time vs PID Output", "Time (s)", "PID Output", ["value"])

    # draw_plot(data_both, "Time vs Encoder and setpoint", "Time (s)", "Values", ["encoder_value", "setpoint_pwm"])
    draw_plot(data_both, "Time vs Encoder, Setpoint, and PID Output", "Time (s)", "Values", ["encoder_value", "setpoint_pwm", "pid_output"])

def measure_encoder():
    motor1 = DC_Motor(motor_in1, motor_in2, motor_ena)
    setpoint_pwm = 0

    frequency = 100 # in Hz
    number_of_measurements = 1000

    encoder = Encoder(left_pin, right_pin)

    start_time = time.time()
    elapsed_time = 0
    data_encoder = []
    data_setpoint = []
    data_both = []

    try:
        for loop_count in range(number_of_measurements):
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

            if loop_count == 50:
                setpoint_pwm = 50
                print("setpoint_pwm = ", setpoint_pwm)
                #motor1.set_speed(setpoint_pwm)
        

    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script
    
    motor1.stop_motor()
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
