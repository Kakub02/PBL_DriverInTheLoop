import time
from mpu6050 import mpu6050
import matplotlib.pyplot as plot
import RPi.GPIO as GPIO

# Set GPIO pin numbers for servos
# Servo roll (boki)
roll_servo_pin = 17
# Servo pitch
pitch_servo_pin = 27

class Servo:
    # servo = Servo(23)
    def __init__(self, pwm_pin):
        self.pwm_pin = pwm_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 50)
        self.pwm.start(0)

    def __del__(self):
        self.pwm.ChangeDutyCycle(7)
        self.pwm.stop()
        GPIO.cleanup() # not sure if it should be here

    def set_duty_cycle(self, duty_cycle):
        self.pwm.ChangeDutyCycle(duty_cycle)

def measure_imu():
    roll_servo = Servo(roll_servo_pin)
    pitch_servo = Servo(pitch_servo_pin)
    setpoint_roll_pwm = 5
    setpoint_pitch_pwm = 5

    frequency = 10  # in Hz
    number_of_measurements = 250

    mpu = mpu6050(0x68)

    start_time = time.time()
    elapsed_time = 0
    data_x = []
    data_y = []
    data_x_setpoint = []
    data_y_setpoint = []

    try:
        for loop_count in range(number_of_measurements):
            roll_servo.set_duty_cycle(setpoint_roll_pwm)
            pitch_servo.set_duty_cycle(setpoint_pitch_pwm)
            print(loop_count)
            accel_data = mpu.get_accel_data()

            data_x.append({"time": elapsed_time, "value": accel_data["x"]})
            data_y.append({"time": elapsed_time, "value": accel_data["y"]})
            data_x_setpoint.append({"time": elapsed_time, "imu_x_value": accel_data["x"], "setpoint_roll_pwm": setpoint_roll_pwm})
            data_y_setpoint.append({"time": elapsed_time, "imu_y_value": accel_data["y"], "setpoint_pitch_pwm": setpoint_pitch_pwm})

            time.sleep(1 / frequency)

            elapsed_time = time.time() - start_time
            if loop_count % 100 == 0:
                setpoint_roll_pwm += 1
                setpoint_pitch_pwm += 1
                roll_servo.set_duty_cycle(setpoint_roll_pwm)
                pitch_servo.set_duty_cycle(setpoint_pitch_pwm)
    except KeyboardInterrupt:
        pass  # Continue with plotting when the user interrupts the script

    draw_plot(data_x, "Time vs Accelerometer X", "Time (s)", "Accel X (chuj wie jaka jednostka)", ["value"])
    draw_plot(data_y, "Time vs Accelerometer Y", "Time (s)", "Accel Y (chuj wie jaka jednostka)", ["value"])
    draw_plot(data_x_setpoint, "Time vs Accelerometer X and setpoint_roll_pwm", "Time (s)", "Values", ["imu_x_value", "setpoint_roll_pwm"])
    draw_plot(data_y_setpoint, "Time vs Accelerometer Y and setpoint_roll_pwm", "Time (s)", "Values", ["imu_y_value", "setpoint_pitch_pwm"])


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
    measure_imu()
