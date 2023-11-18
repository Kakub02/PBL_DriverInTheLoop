from mpu6050 import mpu6050
import time

mpu = mpu6050(0x68)

while True:
    # Accelerometer
    mpu.get_accel_data()
    